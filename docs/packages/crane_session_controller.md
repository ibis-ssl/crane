# crane_session_controller

## 概要

Craneシステムの**最上位制御層**として、試合全体の統括とゲーム状態管理を行うパッケージです。試合状況に応じてプランナープラグインを管理し、ロボットセッションの協調制御を実現します。SSL試合の複雑な状況変化に対応した高レベル制御の中核を担います。

## 主要機能

- **試合統括制御**: SSL試合全体のフロー管理
- **プランナープラグイン管理**: 状況に応じた戦略プランナーの選択・切り替え
- **ゲーム状態管理**: Referee信号に基づく状態遷移制御
- **コンテキスト管理**: 試合状況の履歴・コンテキスト保持
- **YAML設定駆動**: 柔軟な状況対応設定

## アーキテクチャ上の役割

Craneシステムの**最上位制御層**として、SSL Refereeからの指示を解釈し、適切な戦略プランナーを選択・実行することで、試合全体を統括します。プランナープラグインアーキテクチャにより、様々な戦術を状況に応じて動的に切り替えます。

## コンポーネント構成

### SessionController（メイン制御）

- **状態管理**: SSL Referee状態の追跡と対応
- **プランナー選択**: 状況に応じた最適プランナーの選択
- **実行制御**: 選択されたプランナーの実行管理
- **エラーハンドリング**: 異常状況での安全な動作

### Context（コンテキスト管理）

```cpp
class Context {
  // 試合状況の追跡
  GameSituation current_situation;
  std::vector<GameEvent> event_history;

  // 戦略状態
  StrategyMode current_strategy;
  std::map<RobotID, RobotRole> robot_assignments;

  // 設定管理
  YAML::Node situation_configs;
};
```

## 状況対応設定（YAML駆動）

### 基本試合状況

- **HALT.yaml**: 試合停止時の動作
- **STOP.yaml**: 一時停止時の動作
- **INPLAY.yaml**: 通常プレイ時の動作
- **PRE_KICK_OFF.yaml**: キックオフ準備

### 特殊状況

- **OUR_KICKOFF_START.yaml**: 自チームキックオフ
- **OUR_FREE_KICK.yaml**: 自チームフリーキック
- **OUR_PENALTY_KICK.yaml**: 自チームペナルティキック
- **OUR_BALL_PLACEMENT.yaml**: 自チームボール配置

### 敵チーム状況

- **THEIR_FREE_KICK.yaml**: 敵チームフリーキック
- **THEIR_PENALTY_KICK.yaml**: 敵チームペナルティキック
- **THEIR_BALL_PLACEMENT.yaml**: 敵チームボール配置

### 戦術フォーメーション

- **formation.yaml**: 基本フォーメーション
- **ibis_formation.yaml**: ibis特別フォーメーション
- **wing_formation.yaml**: ウイングフォーメーション
- **sandwich.yaml**: サンドイッチ戦術

## 動作フロー

### 状況判定→プランナー選択

```cpp
void SessionController::update() {
  // 1. 現在状況の分析
  auto situation = analyzeCurrent Situation();

  // 2. 適切な設定ファイルの選択
  auto config = loadSituationConfig(situation);

  // 3. プランナーの選択・切り替え
  auto planner = selectOptimalPlanner(config);

  // 4. プランナーの実行
  planner->plan(context_);

  // 5. 結果の統合・配信
  publishRobotCommands();
}
```

### YAML設定例

```yaml
# OUR_FREE_KICK.yaml
situation: "OUR_FREE_KICK"
priority: "high"

robots:
  - id: 0
    role: "kicker"
    planner: "FreekickKickerPlanner"
  - id: 1
    role: "receiver"
    planner: "FreekickReceiverPlanner"
  - id: 2-5
    role: "supporter"
    planner: "FormationPlanner"

constraints:
  ball_distance: 0.5  # SSL規定距離
  formation_type: "freekick_support"
  timeout: 10.0       # タイムアウト時間
```

## プランナーインテグレーション

### プランナー管理

```cpp
class SessionController {
private:
  std::map<std::string, std::shared_ptr<PlannerBase>> planners_;
  std::string current_planner_name_;

public:
  void switchPlanner(const std::string& planner_name) {
    if (planners_.count(planner_name)) {
      current_planner_ = planners_[planner_name];
      current_planner_->reset();
    }
  }
};
```

### 動的プランナー選択

```cpp
std::string selectOptimalPlanner(const GameSituation& situation) {
  if (situation.is_freekick) {
    return situation.our_freekick ? "FreekickOffensePlanner" : "FreekickDefensePlanner";
  } else if (situation.is_penalty) {
    return situation.our_penalty ? "PenaltyOffensePlanner" : "PenaltyDefensePlanner";
  } else {
    return "InplayPlanner";
  }
}
```

## 依存関係

### コア依存

- **crane_planner_plugins**: 実際の戦略プランナー群
- **crane_msg_wrappers**: メッセージ変換・統合
- **crane_msgs**: システムメッセージ定義

### 機能依存

- **yaml-cpp**: YAML設定ファイル処理
- **diagnostic_updater**: システム診断
- **closest_point_vendor**: 幾何学計算支援

## 使用方法

### 基本起動

```bash
# セッションコントローラー起動
ros2 run crane_session_controller crane_session_controller_node

# システム全体起動（含む）
ros2 launch crane_bringup crane.launch.py
```

### カスタム設定

```yaml
# config/custom_situation.yaml
situation: "CUSTOM_DEFENSE"
robots:
  - id: 0
    role: "goalie"
    planner: "GoaliePlanner"
  - id: [1,2,3]
    role: "defender"
    planner: "DefensePlanner"
```

## 最近の開発状況

### 2024年の主要変更

- **YAML設定拡充**: より柔軟な状況対応設定の追加
- **プランナー統合改良**: 新しいプランナープラグインとの統合強化
- **状況判定精度向上**: より正確な試合状況の認識
- **パフォーマンス最適化**: 状態遷移の高速化

### 開発活発度

🟡 **中活動**: システムの最上位制御として、新しい戦術対応や試合規則変更への対応が定期的に行われている。特にYAML設定システムの拡充とプランナー統合機能の改良が活発。

### 技術的特徴

- **設定駆動アーキテクチャ**: コード変更なしでの戦術調整
- **プラグイン統合**: 動的なプランナー選択・切り替え
- **状況適応**: SSL規則変更への柔軟な対応

## パフォーマンス特性

### 応答性能

- **状況判定時間**: <5ms
- **プランナー切り替え**: <10ms
- **全体制御周期**: 60Hz対応

### 管理容量

- **同時管理ロボット**: 最大11台
- **設定ファイル**: 25種類の状況対応
- **プランナー種類**: 20種類以上

## 将来展望

### 技術発展

- **AI統合**: 機械学習による状況判定の高度化
- **適応制御**: 対戦相手に応じた動的戦術調整
- **予測制御**: より長期的な戦略計画

---

**関連パッケージ**: [crane_planner_plugins](./crane_planner_plugins.md) | [crane_play_switcher](./crane_play_switcher.md) | [crane_robot_skills](./crane_robot_skills.md)
