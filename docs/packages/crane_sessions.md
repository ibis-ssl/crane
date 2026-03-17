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

### 守備系セッション

- **GoalieSkillSession** (`goalie_skill`): ゴールキーパー専用戦略
- **DefenderSession** (`defender`): ディフェンダー戦略
- **TotalDefenseSession** (`total_defense`): 統合守備戦略
- **SecondThreatDefenderSession** (`second_threat_defender`): セカンドディフェンダー（シュートコースブロック等）
- **MarkerSession** (`marker`): 敵ロボットマーク戦略

### 攻撃系セッション

- **AttackerSkillSession** (`attacker_skill`): アタッカー戦略
- **SubAttackerSkillSession** (`sub_attacker_skill`): アタッカー支援
- **ForwardSession** (`forward`): フォワードポジショニング（パスコース確保）
- **PassReceiverSession** (`pass_receive`): パスレシーブ戦略

### 特殊状況・セットプレー系セッション

- **SimpleKickOffSkillSession** (`simple_kickoff`): キックオフ戦略
- **OurPenaltyKickSession** (`our_penalty_kick`): 自チームペナルティキック
- **TheirPenaltyKickSession** (`their_penalty_kick`): 敵チームペナルティキック
- **OurDirectFreeKickSession** (`our_direct_free`): 自チーム直接フリーキック
- **CenterStopKickSession** (`center_stop_kick`): センターサークル停止キック（デモ・テスト用）

### ボールプレースメント関連セッション

- **BallPlacementSkillSession** (`ball_placement_skill`): 基本ボール配置
- **PassableBallPlacementSession** (`passable_ball_placement`): パス連携によるボール配置
- **BallPlacementAvoidanceSession** (`ball_placement_avoidance`): 配置エリア回避
- **PlacementTargetPlacerSession** (`placement_target_placer`): 配置目標地点への移動

### フォーメーション・ポジショニング系セッション

- **WingFormationSession** (`wing_formation`): ウィングフォーメーション
- **IbisFormationSession** (`ibis_formation`): ibis基本フォーメーション
- **BallNearByPositionerSkillSession** (`ball_nearby_positioner_skill`): ボール近傍陣形

### ユーティリティセッション

- **WaiterSession** (`waiter`): 待機戦略
- **SimplePlacerSession** (`simple_placer`): エリア配置戦略
- **EmplaceRobotSession** (`emplace_robot`): 指定位置へのロボット配置
- **BallCalibrationDataCollectorSession** (`ball_calibration_data_collector`): キャリブレーションデータ収集
- **TestSession** (`test`): テスト用セッション

## セッションベースアーキテクチャ

### SessionBase基底クラス

```cpp
class SessionBase {
public:
  virtual Status calculateRobotCommand(const std::vector<RobotIdentifier>& robots) = 0;
  // ...
};
```

### セッション生成（SessionFactory）

```cpp
// session_factory.cpp
auto generatePlanner(
  const std::string & session_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  -> SessionBase::SharedPtr
{
  // 登録済みファクトリから生成
  return factory_map.at(session_name)(world_model, node);
}
```

### 登録マップ

```cpp
PLANNER_ENTRY("attacker_skill", AttackerSkillSession),
PLANNER_ENTRY("goalie_skill", GoalieSkillSession),
PLANNER_ENTRY("defender", DefenderSession),
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
# YAML設定による自動選択（unified_session_config.yaml形式）
situations:
  INPLAY:
    description: 通常プレイ
    sessions:
      - name: goalie_skill
        max_robots: 1
      - name: attacker_skill
        max_robots: 1
      - name: defender
        max_robots: 3
```

## 最近の開発状況

### 2025年の主要変更

- **パッケージ統合**: `crane_planner_plugins` を `crane_sessions` へ統合・改称
- **ファクトリ化**: `pluginlib` 依存からの脱却と静的マップによる管理への移行
- **スキル連携強化**: `SkillSession` 系クラスによるスキルパッケージとの密結合化

---

**関連パッケージ**: [crane_session_coordinator](./crane_session_coordinator.md) | [crane_robot_skills](./crane_robot_skills.md) | [crane_game_analyzer](./crane_game_analyzer.md)
