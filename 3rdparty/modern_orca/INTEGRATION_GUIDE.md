# Modern ORCA ライブラリ - Crane統合ガイド

このガイドでは、現在のRVO2実装の代替として、既存のCrane ROS 2 SSLプロジェクトにModern ORCAライブラリを統合する方法を説明します。

## クイック統合

### 1. crane_local_planner/CMakeLists.txtを更新

```cmake
# modern_orca依存関係を追加
find_package(modern_orca REQUIRED)

# 既存のrvo2_plannerターゲットにリンク
target_link_libraries(crane_local_planner
  PRIVATE
    modern_orca::modern_orca
    # ... other dependencies
)
```

### 2. Modern ORCAラッパーを作成

既存の`rvo2_planner.cpp`実装をモダンなラッパーで置き換えます：

```cpp
#include "modern_orca_planner.hpp"
#include <modern_orca/modern_orca.hpp>

class ModernORCAPlanner : public LocalPlannerBase {
public:
    ModernORCAPlanner(rclcpp::Node & node) : LocalPlannerBase("modern_orca_planner", node) {
        // ROSパラメータからの設定
        simulator_ = std::make_unique<modern_orca::CircularAgentSimulator>();
        setupConstraints();
    }

    auto calculateRobotCommand(const crane_msgs::msg::RobotCommands & msg, double theta_offset)
        -> crane_msgs::msg::RobotCommands override {

        updateAgents(msg);
        simulator_->step(1.0 / 60.0);
        return extractResults(msg, theta_offset);
    }

private:
    std::unique_ptr<modern_orca::CircularAgentSimulator> simulator_;
    std::unordered_map<int, modern_orca::AgentId> robot_to_agent_map_;
};
```

### 3. 移行の利点

既存のRVO2実装と比較して、Modern ORCAは以下を提供します：

#### 制約の柔軟性

```cpp
// 旧: overrideTargetPosition()での手動位置オーバーライド
if (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
    while (isInBox(penalty_area, target_pos, PENALTY_AREA_OFFSET)) {
        target_pos += (target_pos - goal_pos).normalized() * 0.05;
    }
}

// 新: 直接速度空間制約
simulator_->addConstraint<SSL_PenaltyAreaConstraint>(
    agent_id, penalty_center, penalty_size, margin
);
```

#### カスタムSSL制約

```cpp
// SSL固有の制約を速度空間に直接追加
class SSL_BallPlacementConstraint : public modern_orca::ConstraintBase<CircularAgent> {
    auto generateHalfPlanes(const CircularAgent& agent, TimeStep dt) const
        -> std::vector<modern_orca::HalfPlaneD> override {
        // 直接半平面制約実装
        // 位置空間回避策よりもはるかに正確
    }
};
```

#### 型安全性

```cpp
// 旧: 整数インデックスによる手動エージェント管理
rvo_sim->setAgentPosition(command.robot_id, RVO::Vector2(pos.x, pos.y));

// 新: 型安全エージェント管理
auto agent_id = simulator_->addAgent(position, preferred_vel, max_speed, radius);
simulator_->getAgent(agent_id).setPosition(new_position);
```

## 高度な統合機能

### 1. カスタムSSL制約

```cpp
// 手動ペナルティエリア回避を適切な制約で置き換え
class CraneSSLConstraints {
public:
    static void addAllSSLConstraints(modern_orca::CircularAgentSimulator& sim,
                                   const WorldModel& world_model) {

        // ゲーム状態認識付きのボール回避
        auto ball_distance = getSSLBallDistance(world_model.getGameState());
        sim.addGlobalConstraint<SSL_BallAvoidanceConstraint>(
            world_model.ball().pos, ball_distance);

        // 動的ペナルティエリア制約
        sim.addGlobalConstraint<SSL_PenaltyAreaConstraint>(
            world_model.getOurPenaltyArea(),
            getSSLPenaltyMargin(world_model.getGameState()));

        // ボール配置エリア回避
        if (auto placement_area = world_model.getBallPlacementArea()) {
            sim.addGlobalConstraint<SSL_BallPlacementConstraint>(*placement_area);
        }
    }
};
```

### 2. パフォーマンス改善

```cpp
// より良いパフォーマンスのために並列処理を有効化
simulator_->setParallelExecution(true);

// より良い速度解のために最適ソルバーを使用
auto optimal_solver = std::make_unique<modern_orca::OptimalLinearProgram2DSolver>(max_speed);
simulator_->setSolver(std::move(optimal_solver));
```

### 3. 既存コードとの統合

```cpp
// crane型とmodern_orca型間の変換
modern_orca::Vector2D toModernORCA(const crane::Point& point) {
    return {point.x(), point.y()};
}

crane::Point fromModernORCA(const modern_orca::Vector2D& vec) {
    return crane::Point(vec.x(), vec.y());
}

// 既存のロボットコマンド構造との互換性を維持
crane_msgs::msg::RobotCommands extractResults(
    const crane_msgs::msg::RobotCommands& original_commands,
    double theta_offset) {

    crane_msgs::msg::RobotCommands result;
    for (const auto& cmd : original_commands.robot_commands) {
        auto agent_id = robot_to_agent_map_[cmd.robot_id];
        auto velocity = simulator_->getAgent(agent_id).velocity();

        // 元のコマンド形式に戻す変換
        crane_msgs::msg::RobotCommand new_cmd = cmd;
        new_cmd.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
        // ... 速度フィールドを設定

        result.robot_commands.push_back(new_cmd);
    }
    return result;
}
```

## 移行戦略

### フェーズ1: ドロップイン置換

- RVO2ライブラリ呼び出しをModern ORCA同等品で置換
- 最初は既存の制約ロジックを維持
- パフォーマンスと動作の一致を検証

### フェーズ2: 制約移行

- 位置空間制約を速度空間制約に変換
- SSL固有の制約クラスを実装
- 手動位置オーバーライドロジックを削除

### フェーズ3: 最適化

- 並列処理を有効化
- 最適ソルバーを使用
- 高度な動作のためのカスタム制約を追加

## パフォーマンス比較

| 指標 | 元のRVO2 | Modern ORCA |
|------|----------|-------------|
| 制約タイプ | 1個（ORCAのみ） | 無制限拡張可能 |
| 制約精度 | 位置空間回避策 | 直接速度空間 |
| 型安全性 | 手動インデックス管理 | コンパイル時型安全性 |
| 並列処理 | なし | あり（OpenMP） |
| カスタム制約 | ライブラリ修正が必要 | プラグインアーキテクチャ |
| メモリ安全性 | 手動メモリ管理 | スマートポインタによるRAII |
| SSL統合 | 手動位置ハック | 直接制約API |

## テスト

```cpp
// 包括的なテストスイートが含まれています
#include <modern_orca/modern_orca.hpp>

TEST_CASE("SSL Integration", "[ssl]") {
    modern_orca::CircularAgentSimulator sim;

    // SSL固有の制約をテスト
    auto agent = sim.addAgent(Vector2D{0, 0}, Vector2D{1, 0}, 2.0, 0.09);
    sim.addConstraint<SSL_BallAvoidanceConstraint>(agent, Vector2D{0.3, 0}, 0.5);

    sim.step(1.0/60.0);

    // ボール回避動作を検証
    REQUIRE(distance(sim.getAgent(agent).position(), Vector2D{0.3, 0}) >= 0.5);
}
```

Modern ORCAライブラリは、Craneのアーキテクチャとの完全な互換性を維持しながら、既存のRVO2実装に対する完全で型安全かつ拡張可能な代替手段を提供します。
