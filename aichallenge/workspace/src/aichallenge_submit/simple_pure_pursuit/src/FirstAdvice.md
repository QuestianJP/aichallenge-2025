ありがとうございます。`simple_pure_pursuit.cpp` のコードを確認しました。非常に丁寧に作り込まれており、現状のPure Pursuitの処理は以下のような構成になっています：

---

## ✅ 現在の処理の要点（Pure Pursuit）

1. **現在位置と予測位置（x秒後）を計算**
2. **それぞれに対してルックアヘッドポイントを探索**
3. **2つのルックアヘッドに対してPure Pursuitの操舵角を計算**
4. **操舵角の平均を指令値として使用**
5. **操舵角に応じた速度制限も実装済み**

この構成は、**予測補正付きPure Pursuit**として非常に工夫されています。ただし、操舵の追従性が悪い場合、**Pure Pursuit特有の「目標点に向かう」挙動が振動を引き起こす**ことがあります。

---

## 🧠 Stanley法への変更ポイント

Stanley法では、以下の2つの誤差を使って操舵角を計算します：

- **横方向誤差（クロストラックエラー）**
- **方位角誤差（ヘディングエラー）**

Stanley法の操舵角計算式は：

$$
\delta = \theta_e + \arctan\left(\frac{k \cdot e}{v}\right)
$$

### 🔧 変更すべきポイント

以下の部分を中心に変更します：

---

### ① **操舵角の計算部分**

#### 現在のコード（Pure Pursuit）：
```cpp
double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) - yaw;
double steering_tire_angle = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
```

#### Stanley法に変更：
```cpp
double dx = lookahead_point_x - rear_x;
double dy = lookahead_point_y - rear_y;
double path_yaw = std::atan2(dy, dx);
double theta_e = path_yaw - yaw;
double e = calcLateralDeviation(lookahead_point_itr->pose, odometry_->pose.pose); // 横方向誤差
double steering_tire_angle = theta_e + std::atan2(k_gain * e, current_longitudinal_vel);
```

- `calcLateralDeviation` は既に `tier4_autoware_utils` に含まれているので使えます。
- `k_gain` は新たに Stanley用のゲインとしてパラメータ化すると良いです。

---

### ② **予測位置を使う場合**

予測位置に対しても同様に Stanley法を適用できます：

```cpp
double dx2 = lookahead_point2_x - predicted_rear_x;
double dy2 = lookahead_point2_y - predicted_rear_y;
double path_yaw2 = std::atan2(dy2, dx2);
double theta_e2 = path_yaw2 - predicted_yaw;
double e2 = calcLateralDeviation(lookahead_point2_itr->pose, predicted_pos); // 予測位置に対する横方向誤差
double steering_tire_angle2 = theta_e2 + std::atan2(k_gain * e2, current_longitudinal_vel);
```

---

### ③ **操舵角の統合**

現在と予測の操舵角を平均する処理はそのまま活かせます：

```cpp
cmd.lateral.steering_tire_angle = steering_tire_angle_gain_ * (steering_tire_angle + steering_tire_angle2) / 2.0;
```

---

### ④ **パラメータ追加（Stanley用）**

```cpp
stanley_gain_ = declare_parameter<float>("stanley_gain", 1.0);
```

---

## ✅ 次のステップ

Stanley法への変更は、**操舵角の計算式を差し替えるだけで実現可能**です。  
ご希望であれば、変更済みの `onTimer()` 関数のコードを丸ごと提供できます。

また、Stanley法のゲイン調整や、振れ幅のログ出力なども追加できます。  
どうしますか？コード全体の変更案を提示しましょうか？
