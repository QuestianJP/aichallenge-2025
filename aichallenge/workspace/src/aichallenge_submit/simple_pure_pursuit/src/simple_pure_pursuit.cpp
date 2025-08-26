#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <tuple>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

namespace
{
inline double clamp(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }

struct LookaheadResult
{
  geometry_msgs::msg::Point point;
  size_t base_index;     // segment start index i (point i -> i+1)
  double segment_ratio;  // interpolation t in [0,1]
  bool valid{false};
};

// compute lookahead point along trajectory
LookaheadResult computeLookaheadPoint(
  const Trajectory & traj, size_t start_idx, double Ld, double rear_x, double rear_y,
  size_t max_forward_points = 500)
{
  LookaheadResult res;
  if (traj.points.size() < 2) return res;

  size_t i = std::min(start_idx, traj.points.size() - 2);

  auto & Pi = traj.points[i].pose.position;
  double dx0 = Pi.x - rear_x, dy0 = Pi.y - rear_y;
  double acc = std::hypot(dx0, dy0);

  size_t end_limit = std::min(traj.points.size() - 1, i + max_forward_points);
  for (; i < end_limit; ++i) {
    const auto & P0 = traj.points[i].pose.position;
    const auto & P1 = traj.points[i + 1].pose.position;
    double seg = std::hypot(P1.x - P0.x, P1.y - P0.y);
    if (seg < 1e-6) continue;

    if (acc + seg >= Ld) {
      double remain = Ld - acc;
      double t = clamp(remain / seg, 0.0, 1.0);
      geometry_msgs::msg::Point p;
      p.x = P0.x + t * (P1.x - P0.x);
      p.y = P0.y + t * (P1.y - P0.y);
      p.z = P0.z + t * (P1.z - P0.z);

      res.point = p;
      res.base_index = i;
      res.segment_ratio = t;
      res.valid = true;
      return res;
    }
    acc += seg;
  }

  // if we run out of points
  res.point = traj.points.back().pose.position;
  res.base_index = traj.points.size() - 2;
  res.segment_ratio = 1.0;
  res.valid = true;
  return res;
}

// nearest forward index (point ahead of vehicle heading)
size_t findNearestForwardIndex(const Trajectory & traj, const geometry_msgs::msg::Pose & pose)
{
  if (traj.points.empty()) return 0;

  size_t idx = findNearestIndex(traj.points, pose.position);

  const double yaw = tf2::getYaw(pose.orientation);
  const double cos_y = std::cos(yaw), sin_y = std::sin(yaw);

  for (size_t k = 0; k + idx + 1 < traj.points.size(); ++k) {
    const auto & p = traj.points[idx].pose.position;
    const double dx = p.x - pose.position.x;
    const double dy = p.y - pose.position.y;
    double along = dx * cos_y + dy * sin_y;
    if (along >= 0.0) break;
    ++idx;
  }
  return std::min(idx, traj.points.size() - 1);
}

}  // namespace

// ==========================
// Constructor
// ==========================
SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // parameters
  wheel_base_(declare_parameter<double>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<double>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<double>("lookahead_min_distance", 1.0)),
  lookahead_max_distance_(declare_parameter<double>("lookahead_max_distance", 8.0)),
  speed_proportional_gain_(declare_parameter<double>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<double>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<double>("steering_tire_angle_gain", 1.0)),
  // tuned feedback gains
  k_lat_err_(declare_parameter<double>("k_lat_err", 0.3)),   // was 0.15
  k_head_err_(declare_parameter<double>("k_head_err", 1.0)), // was 0.6
  max_abs_steer_(declare_parameter<double>("max_abs_steer", 0.61))  // ~35 degrees
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimplePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

// ==========================
// Timer Callback
// ==========================
void SimplePurePursuit::onTimer()
{
  if (!subscribeMessageAvailable()) return;

  const auto & odom = *odometry_;
  const auto & traj = *trajectory_;

  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  // ---- longitudinal control ----
  size_t nearest_idx = findNearestForwardIndex(traj, odom.pose.pose);
  const TrajectoryPoint & nearest_pt = traj.points.at(nearest_idx);

  const double target_v =
    use_external_target_vel_ ? external_target_vel_ : nearest_pt.longitudinal_velocity_mps;
  const double current_v = odom.twist.twist.linear.x;

  cmd.longitudinal.speed = target_v;
  cmd.longitudinal.acceleration = speed_proportional_gain_ * (target_v - current_v);

  // ---- lateral control ----
  const double yaw = tf2::getYaw(odom.pose.pose.orientation);
  const double rear_x = odom.pose.pose.position.x - 0.5 * wheel_base_ * std::cos(yaw);
  const double rear_y = odom.pose.pose.position.y - 0.5 * wheel_base_ * std::sin(yaw);

  const double path_yaw = tf2::getYaw(nearest_pt.pose.orientation);
  const double epsi = tier4_autoware_utils::normalizeRadian(path_yaw - yaw);

  // dynamic lookahead distance
  double Ld = lookahead_gain_ * std::max(std::fabs(target_v), 0.0) + lookahead_min_distance_;
  const double shrink = clamp(std::fabs(epsi) / (M_PI / 3.0), 0.0, 0.6); // allow up to 60% shrink
  Ld = clamp(Ld * (1.0 - shrink), lookahead_min_distance_, lookahead_max_distance_);

  const auto lh = computeLookaheadPoint(traj, nearest_idx, Ld, rear_x, rear_y);
  if (!lh.valid) {
    pub_cmd_->publish(cmd);
    pub_raw_cmd_->publish(cmd);
    return;
  }

  // debug marker
  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point = lh.point;
  pub_lookahead_point_->publish(lookahead_point_msg);

  // pure pursuit geometry
  const double dx = lh.point.x - rear_x;
  const double dy = lh.point.y - rear_y;
  const double alpha = std::atan2(dy, dx) - yaw;
  const double kappa_pp = (2.0 * std::sin(alpha)) / std::max(Ld, 1e-3);
  double delta_pp = std::atan(wheel_base_ * kappa_pp);

  // feedback correction
  const auto & nearest_pose = nearest_pt.pose;
  const double ey = calcLateralDeviation(nearest_pose, odom.pose.pose.position);
  double delta_fb = k_lat_err_ * ey + k_head_err_ * epsi;

  double delta = steering_tire_angle_gain_ * delta_pp + delta_fb;

  // steering saturation
  delta = clamp(delta, -max_abs_steer_, max_abs_steer_);
  if (std::fabs(current_v) < 1.0) {
    delta = clamp(delta, -0.3, 0.3); // reduce oscillations at very low speed
  }

  // publish
  cmd.lateral.steering_tire_angle = delta;
  pub_cmd_->publish(cmd);

  // raw PP for debugging
  AckermannControlCommand raw = cmd;
  raw.lateral.steering_tire_angle = delta_pp;
  pub_raw_cmd_->publish(raw);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "trajectory is not available");
    return false;
  }
  if (trajectory_->points.size() < 2) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "trajectory requires >= 2 points");
    return false;
  }
  return true;
}

}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
