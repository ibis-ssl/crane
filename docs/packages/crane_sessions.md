# crane_sessions

## 概要

Craneシステムの**戦略タクティックプラグインコレクション**として、様々な試合状況に対応した戦略実装を提供するパッケージです。`crane_planner_plugins`（旧称）から改称・統合されました。ファクトリパターンにより、攻撃・守備・特殊状況の戦略を柔軟に組み合わせ、動的な戦術実行を実現します。

## 主要機能

- **ファクトリベースの生成**: `SessionFactory`による動的戦略生成
- **多様な戦略実装**: 20種類以上の専門タクティック
- **状況適応制御**: 試合状況に応じた戦略自動選択
- **ロボット協調**: マルチロボット戦術の統合実行
- **リアルタイム計画**: 高速な戦術決定と実行

## アーキテクチャ上の役割

Craneシステムの**戦術戦略層**として、`crane_session_coordinator`からの指示を受けて具体的な戦術を実行し、`crane_robot_skills`に個別ロボット行動を指示します。

## タクティックプラグイン一覧

（カッコ内は設定ファイルで使用するキー名）

### 守備系タクティック

- **GoalieSkillTactic** (`goalie_skill`): ゴールキーパー専用戦略
- **DefenderTactic** (`defender`): ディフェンダー戦略
- **TotalDefenseTactic** (`total_defense`): 統合守備戦略
- **SecondThreatDefenderTactic** (`second_threat_defender`): セカンドディフェンダー（シュートコースブロック等）
- **MarkerTactic** (`marker`): 敵ロボットマーク戦略

### 攻撃系タクティック

- **AttackerSkillTactic** (`attacker_skill`): アタッカー戦略
- **SubAttackerSkillTactic** (`sub_attacker_skill`): アタッカー支援
- **ForwardTactic** (`forward`): フォワードポジショニング（パスコース確保）
- **PassReceiverTactic** (`pass_receive`): パスレシーブ戦略

### 特殊状況・セットプレー系タクティック

- **SimpleKickOffSkillTactic** (`simple_kickoff`): キックオフ戦略
- **OurPenaltyKickTactic** (`our_penalty_kick`): 自チームペナルティキック
- **TheirPenaltyKickTactic** (`their_penalty_kick`): 敵チームペナルティキック
- **OurDirectFreeKickTactic** (`our_direct_free`): 自チーム直接フリーキック
- **CenterStopKickTactic** (`center_stop_kick`): センターサークル停止キック（デモ・テスト用）

### ボールプレースメント関連タクティック

- **BallPlacementSkillTactic** (`ball_placement_skill`): 基本ボール配置
- **PassableBallPlacementTactic** (`passable_ball_placement`): パス連携によるボール配置
- **BallPlacementAvoidanceTactic** (`ball_placement_avoidance`): 配置エリア回避
- **PlacementTargetPlacerTactic** (`placement_target_placer`): 配置目標地点への移動
- **PlacementTargetNearByPositionerSkillTactic** (`placement_target_nearby_positioner_skill`): 配置目標近傍待機

### フォーメーション・ポジショニング系タクティック

- **WingFormationTactic** (`wing_formation`): ウィングフォーメーション
- **IbisFormationTactic** (`ibis_formation`): ibis基本フォーメーション
- **BallNearByPositionerSkillTactic** (`ball_nearby_positioner_skill`): ボール近傍陣形

### ユーティリティタクティック

- **WaiterTactic** (`waiter`): 待機戦略
- **SimplePlacerTactic** (`simple_placer`): エリア配置戦略
- **EmplaceRobotTactic** (`emplace_robot`): 指定位置へのロボット配置
- **BallCalibrationDataCollectorTactic** (`ball_calibration_data_collector`): キャリブレーションデータ収集
- **TestTactic** (`test`): テスト用タクティック

## タクティックベースアーキテクチャ

### TacticBase基底クラス

```cpp
class SessionBase {
public:
  virtual Status calculateRobotCommand(const std::vector<RobotIdentifier>& robots) = 0;
  // ...
};
```

### タクティック生成（SessionFactory）

```cpp
// tactic_factory.cpp
auto generatePlanner(
  const std::string & tactic_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  -> SessionBase::SharedPtr
{
  // 登録済みファクトリから生成
  return factory_map.at(tactic_name)(world_model, node);
}
```

### 登録マップ

```cpp
PLANNER_ENTRY("attacker_skill", AttackerSkillTactic),
PLANNER_ENTRY("goalie_skill", GoalieSkillTactic),
PLANNER_ENTRY("defender", DefenderTactic),
// ...
```

## 依存関係

### コア依存

- **crane_robot_skills**: 個別ロボット行動の実行
- **crane_msg_wrappers**: メッセージラッパー
- **crane_geometry**: 幾何学計算ライブラリ
- **crane_physics**: 物理計算・ボールモデル

### 使用方法

### Session Controllerからの利用

`crane_session_coordinator` の設定ファイルで名前を指定します。

```yaml
# YAML設定による自動選択
robots:
  - id: 0
    role: "goalie"
    tactic: "goalie_skill"
  - id: 1
    role: "attacker"
    tactic: "attacker_skill"
  - id: [2,3,4]
    role: "defender"
    tactic: "defender"
```

## 最近の開発状況

### 2025年の主要変更

- **パッケージ統合**: `crane_sessions` を `crane_sessions` へ統合・改称
- **ファクトリ化**: `pluginlib` 依存からの脱却と静的マップによる管理への移行
- **スキル連携強化**: `SkillTactic` 系クラスによるスキルパッケージとの密結合化

---

**関連パッケージ**: [crane_session_coordinator](./crane_session_coordinator.md) | [crane_robot_skills](./crane_robot_skills.md) | [crane_game_analyzer](./crane_game_analyzer.md)
