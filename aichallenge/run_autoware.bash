#!/bin/bash
cleanup_zenoh() {
    echo "Zenoh bridge cleanup..."
    pkill -f "zenoh-bridge-ros2dds" 2>/dev/null || true
    sleep 0.5
}
mode="${1}"

case "${mode}" in
"awsim")
    opts=("simulation:=true" "use_sim_time:=true" "run_rviz:=true")
    ;;
"awsim-no-viz")
    opts=("simulation:=true" "use_sim_time:=true" "run_rviz:=false")
    ;;
"vehicle")
    zenoh-bridge-ros2dds -c /vehicle/zenoh.json5 &
    trap cleanup_zenoh EXIT SIGINT SIGTERM
    opts=("simulation:=false" "use_sim_time:=false" "run_rviz:=false")
    ;;
"rosbag")
    opts=("simulation:=false" "use_sim_time:=true" "run_rviz:=true")
    ;;
*)
    echo "invalid argument (use 'awsim' or 'vehicle' or 'rosbag')"
    exit 1
    ;;
esac

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
sudo setfacl -m u:"$(whoami)":r /dev/cpu/*/msr
sudo setcap cap_sys_rawio=ep /autoware/install/system_monitor/lib/system_monitor/msr_reader
/autoware/install/system_monitor/lib/system_monitor/msr_reader

ros2 launch aichallenge_system_launch aichallenge_system.launch.xml "${opts[@]}"
