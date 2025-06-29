# Modern ORCA ライブラリ - Crane統合ガイド

このガイドでは、現在のRVO2実装の代替として、既存のCrane ROS 2 SSLプロジェクトにModern ORCAライブラリを統合する方法を説明します。

## 統合ステータス

✅ **統合完了** - Modern ORCAプランナーは既に実装され、動作しています。

## 現在の実装

### 1. CMakeLists.txt統合

現在のCMakeLists.txtは既にModern ORCA統合をサポートしています：

```cmake
# modern_orca は ament_auto_find_build_dependencies() で自動発見
ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/rvo2_planner.cpp
  src/modern_orca_planner.cpp  # <- Modern ORCA実装
  src/crane_local_planner.cpp
)
```

### 2. Modern ORCAプランナー実装

`ModernORCAPlanner`クラスは既に実装済みです：

```cpp
#include <modern_orca/modern_orca.hpp>
#include <modern_orca/ssl_constraints/ssl_constraint_manager.hpp>

class ModernORCAPlanner : public LocalPlannerBase {
public:
    explicit ModernORCAPlanner(rclcpp::Node & node);

    auto calculateRobotCommand(const crane_msgs::msg::RobotCommands & msg, double theta_offset)
        -> crane_msgs::msg::RobotCommands override;

private:
    std::unique_ptr<modern_orca::SSLConstraintManagerForCircularAgent> ssl_constraint_manager_;
    std::unordered_map<uint32_t, std::unique_ptr<modern_orca::CircularAgent>> agents_;
};
```

### 3. 実装済み機能

現在のModern ORCA実装では以下の機能が利用可能です：

#### SSL専用制約システム

```cpp
// SSL制約マネージャーによる統合制約管理
ssl_constraint_manager_ = std::make_unique<modern_orca::SSLConstraintManagerForCircularAgent>();

// ゲーム状況に応じた自動制約調整
ssl_constraint_manager_->updateFromWorldModel(world_model);
ssl_constraint_manager_->updateFromRefereeCommand(referee_command);
ssl_constraint_manager_->applyAutomaticConstraintAdjustments();
```

#### エージェントベースのアーキテクチャ

```cpp
// 各ロボットを個別のエージェントとして管理
std::unordered_map<uint32_t, std::unique_ptr<modern_orca::CircularAgent>> agents_;

// 動的半径計算（速度に応じて調整）
double velocity_norm = velocity.norm();
double dynamic_radius = ORCA_RADIUS + velocity_norm * 0.1;
```

#### 高度な速度プロファイル制御

```cpp
// 台形速度プロファイルによる位置制御
Vector2 calculateTrapezoidalVelocityProfile(
    const crane_msgs::msg::RobotCommand & command,
    const Point & current_position);
```

## 現在利用可能な機能

### 1. SSL専用制約システム

制約マネージャーが以下の制約を自動管理します：

```cpp
// 利用可能な制約類型
enum class SSLConstraintType {
    BALL_AVOIDANCE,              // ボール回避
    PENALTY_AREA_AVOIDANCE,      // ペナルティエリア回避
    BALL_PLACEMENT_AVOIDANCE,    // ボール配置エリア回避
    REFEREE_COMMAND,             // レフェリーコマンド対応
    ROBOT_COLLISION              // ロボット間衝突回避
};

// 制約の有効/無効化
ssl_constraint_manager_->setConstraintEnabled(SSLConstraintType::BALL_AVOIDANCE, true);
```

### 2. デバッグ可視化

```cpp
// 制約可視化パラメータ
node.declare_parameter("debug_visualize_constraints", false);
node.declare_parameter("debug_visualize_orca_lines", false);
node.declare_parameter("debug_show_performance_metrics", false);
```

### 3. パラメータ設定

```cpp
// ORCA固有パラメータ
node.declare_parameter("orca_time_step", 0.1);
node.declare_parameter("orca_neighbor_dist", 15.0);
node.declare_parameter("orca_max_neighbors", 10);
node.declare_parameter("orca_time_horizon", 2.0);
node.declare_parameter("orca_radius", 0.05);
node.declare_parameter("orca_max_speed", 4.0);
```

## 使用方法

### 1. プランナーの有効化

Local Plannerコンポーネントで`ModernORCAPlanner`を選択：

```cpp
// プランナーの初期化
auto planner = std::make_shared<crane::ModernORCAPlanner>(node);
```

### 2. パラメータ調整

ROSパラメータファイルで設定を調整：

```yaml
local_planner_node:
  ros__parameters:
    # 基本パラメータ
    max_vel: 4.0
    max_acc: 4.0

    # ORCA固有パラメータ
    orca_time_step: 0.1
    orca_neighbor_dist: 15.0
    orca_max_neighbors: 10
    orca_time_horizon: 2.0
    orca_radius: 0.05
    orca_max_speed: 4.0

    # 制約設定
    constraint_ball_avoidance_enabled: true
    constraint_penalty_area_avoidance_enabled: true
    constraint_ball_placement_avoidance_enabled: true
    constraint_referee_command_enabled: true
    constraint_robot_collision_enabled: true
```

### 3. デバッグ可視化の有効化

```yaml
# デバッグ可視化
debug_visualize_constraints: true
debug_visualize_orca_lines: true
debug_show_performance_metrics: true
```

## RVO2からModern ORCAへの改善点

| 項目 | RVO2 | Modern ORCA |
|------|------|-------------|
| 制約システム | 基本ORCA制約のみ | SSL専用制約システム |
| 制約管理 | 手動管理 | 自動制約マネージャー |
| 型安全性 | C風配列アクセス | スマートポインタ/RAII |
| SSL統合 | 位置空間での手動回避 | 速度空間での直接制約 |
| デバッグ | 限定的 | 包括的可視化システム |
| パラメータ管理 | 固定値 | 動的ROSパラメータ |
| エージェント管理 | インデックスベース | 型安全IDシステム |
| 速度プロファイル | 基本的 | 高度な台形プロファイル |

## テスト

現在のテストスイートは以下で利用可能です：

```bash
# Modern ORCAプランナーのテスト実行
colcon test --packages-select crane_local_planner --event-handlers console_cohesion+ --ctest-args -R test_modern_orca_planner
```

テストファイル: `crane_local_planner/test/test_modern_orca_planner.cpp`

## まとめ

Modern ORCAライブラリは既にCraneプロジェクトに統合済みで、RVO2の代替として動作しています。SSL固有の制約システム、高度なデバッグ機能、型安全なエージェント管理を提供し、RoboCup SSLの要件に特化した最適化されたソリューションです。
