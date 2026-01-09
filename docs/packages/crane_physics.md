# crane_physics

## Header-only physics simulation and robot modeling utility package

Craneシステムの**物理計算・シミュレーションライブラリ**として、ボール物理モデル、ロボット運動学、PID制御、軌道計算などの物理・制御系機能を提供するパッケージです。リアルタイム（60-100Hz）制御ループに最適化された設計で、高精度な戦略立案を支えています。

## 主要機能

### 🏀 ボール物理モデル

- **3D状態ベースシミュレーション**: STOPPED/ROLLING/FLYING状態の自動遷移
- **高度な軌道予測**: 重力・空気抵抗を考慮した放物運動計算
- **物理パラメータ**: 0.5 m/s²減速度、設定可能な重力定数
- **予測メソッド**: 位置・速度予測、停止時間計算

### 🤖 ロボット動力学

- **運動プロファイル**: 最適移動時間のための台形速度プロファイル
- **物理定数**: 60mmロボット半径、90mmドリブラー距離
- **移動時間計算**: 簡単・高度なモーション計画サポート
- **ボール接触検知**: マイクロ秒精度の接触タイミング

### 🎯 戦略ツール

- **ハンガリアンアルゴリズム**: 最適ロボット-目標割り当て（O(n³)）
- **パス解析**: 障害物検出とチップキック推奨
- **PID制御器**: 積分ワインドアップ保護付きリアルタイム制御
- **ターゲット ジオメトリ**: 動的目標指定システム

## アーキテクチャ上の役割

Craneシステムの**物理シミュレーション層**として、ボールやロボットの動きを予測し、戦略立案や経路計画に必要な物理計算を提供します。リアルタイムでの高精度な予測により、効果的なプレイ実行を可能にしています。

## コアコンポーネント

### ボール物理モデル (`ball_info.hpp`)

```cpp
struct Ball {
    double deceleration = 0.5;    // 転がり減速度 (m/s²)
    double gravity = -9.81;       // 重力加速度 (m/s²)
    Point pos, vel;               // 2D位置・速度
    double pos_z, vel_z;          // 3D高度・垂直速度
    State state;                  // STOPPED/ROLLING/FLYING
};
```

**主要メソッド**:

- `getPredictedPosition(time_ahead)` - 未来のボール位置
- `getStopTime()` - ボール停止までの時間
- `isMovingTowards(target)` - 方向性運動解析

### 物理状態遷移システム

**ROLLING物理**:

- 一定減速度モデル使用
- 停止時間予測: `t_stop = v / deceleration`
- 最大距離: `d_max = v²/(2·deceleration)`

**FLYING物理（放物運動）**:

- 重力付き3D軌道計算
- 地面交点検出
- 飛行→転がり状態のシームレス遷移

**状態遷移**:

- 高度・速度閾値に基づく自動状態検出
- 振動防止のヒステリシスベース切り替え

### ロボット情報管理 (`robot_info.hpp`)

```cpp
struct RobotInfo {
    uint8_t id;                    // ロボット識別子
    Pose2D pose;                   // 位置と向き
    Velocity2D vel;                // 線形・角速度
    BallContact ball_contact;      // ボール接触検出
};
```

**物理定数**:

- ロボット半径: 0.060m（衝突検出用）
- ドリブラー距離: 0.090m（中心からボール接触点）

### 移動時間計算 (`travel_time.hpp`)

**台形運動プロファイル**:

1. **加速フェーズ**: `t₁ = (v_max - v₀) / a`
2. **巡航フェーズ**: `t₂ = d_remaining / v_max`
3. **減速フェーズ**: `t₃ = v_max / a`

```cpp
double getTravelTimeTrapezoidal(
    std::shared_ptr<RobotInfo> robot,
    Point target,
    double max_acceleration,
    double max_velocity
);
```

### PID制御器 (`pid_controller.hpp`)

```cpp
class PIDController {
    double setGain(double p, double i, double d, double max_int = -1.0);
    double update(double error, double dt);
};
```

**機能**:

- 積分ワインドアップ保護付き標準PID制御
- ロボット運動制御に適した高精度制御

### 戦略計算ツール

**ハンガリアンアルゴリズム** (`position_assignments.hpp`):

- **計算量**: O(n³)でのハンガリアンアルゴリズム
- **最適化**: 総ユークリッド距離最小化
- **用途**: フォーメーション制御、マルチロボット協調

**パス解析** (`pass.hpp`):

- 通行路構築とチップキック判定
- 敵ロボットの妨害距離計算
- 必要チップ距離の推奨

## 統合ポイント

### crane_geometryとの連携

```cpp
using Point = Eigen::Vector2d;          // 2D位置
using Point3D = EigenVector3d;        // 3D位置
using Velocity2D = Eigen::Vector2d;     // 2D速度
using Segment = bg::model::segment<Point>;  // 線分
using Circle = crane::geometry::model::Circle<Point>;  // 円
```

### Craneシステムでの使用

- **crane_world_model_publisher**: 状態推定・ボール追跡
- **crane_robot_skills**: モーション実行タイミング
- **crane_local_planner**: 経路計画・軌道生成
- **crane_tactic_coordinator**: マルチロボット協調

## パフォーマンス特性

### 計算複雑度

- **ボール予測**: 単純状態でO(1)、軌道サンプリングでO(n)
- **ハンガリアンアルゴリズム**: nロボットでO(n³)
- **PID制御器**: 制御サイクル毎にO(1)
- **パス解析**: m敵ロボットでO(m)

### リアルタイム性能

- **設計対象**: 60-100Hz制御ループ
- **決定的計算時間**: 予測可能な処理時間
- **組み込み適用**: ロボット制御器での実行に適用

## テスト

包括的な単体テスト：

- **ボール物理**: 状態遷移、軌道予測、放物運動
- **ロボット動力学**: 移動時間計算、運動プロファイル
- **PID制御器**: ゲイン設定、積分クランプ、制御出力
- **位置割り当て**: ハンガリアンアルゴリズム最適化

```bash
colcon test --packages-select crane_physics
```

## 物理定数・キャリブレーション

### ボール物理パラメータ

- **減速度**: 0.5 m/s²（芝フィールドでの転がり摩擦）
- **重力**: -9.81 m/s²（標準地球重力）
- **高度閾値**: 0.05m（飛行vs転がり検出）
- **速度閾値**: 0.1 m/s（移動），0.05 m/s（停止）

### ロボット物理特性

- **ロボット半径**: 0.060m（90mm規則準拠）
- **ドリブラー距離**: 0.090m（中心からボール接触）
- **最大速度**: ~4.0 m/s（典型的ロボット性能）
- **最大加速度**: ~2.0 m/s²（典型的ロボット性能）

---

**関連パッケージ**: [crane_geometry](./crane_geometry.md) | [crane_world_model_publisher](./crane_world_model_publisher.md) | [crane_local_planner](./crane_local_planner.md)
