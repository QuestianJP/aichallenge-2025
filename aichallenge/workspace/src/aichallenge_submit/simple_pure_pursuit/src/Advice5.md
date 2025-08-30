こちらがご要望の3つの機能を追加した **Stanley法強化版ソースコード** です：

📄 [Stanley法強化版 simple_pure_pursuit_stanley.cpp](blob:https://m365.cloud.microsoft/c25ebeef-d27a-4478-983a-37451cc5837f)

---

## ✅ 追加された機能一覧

### ① **軌道の曲率計算関数 `calcPathCurvature()`**
- ルックアヘッドポイントの前後の軌道点から曲率を推定。
- 曲率 $$\kappa = \frac{1}{R}$$ を算出。

### ② **Stanleyゲインの動的調整**
```cpp
double curvature = calcPathCurvature(lookahead_point_itr);
double dynamic_gain = stanley_gain_ + curvature_gain_ * curvature;
```
- `curvature_gain_` は新たなパラメータとして追加。

### ③ **フィードフォワード項の追加**
```cpp
double ff_angle = std::atan(wheel_base_ * curvature);
steering_tire_angle += ff_angle;
```
- 曲率に応じた先回り操舵を追加。

---

## 🔧 パラメータ追加例（`declare_parameter`）
```cpp
curvature_gain_ = declare_parameter<float>("curvature_gain", 2.0);
```

---

## 🧪 次のステップ

1. `stanley_gain` と `curvature_gain` をチューニング（例：1.0〜3.0）
2. 実機またはシミュレータで走行テスト
3. ログで振れ幅・追従性・ラップタイムを評価

---

必要であれば、**曲率計算の詳細説明**や**ログ可視化ツール**も提供できます。  
次に進めたい内容はありますか？
