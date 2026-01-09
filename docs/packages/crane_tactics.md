# crane_tactics

## 概要

Craneシステムの**戦略プランナープラグインコレクション**として、様々な試合状況に対応した戦略実装を提供するパッケージです。`crane_tactics`（旧称）から改称・統合されました。ファクトリパターンにより、攻撃・守備・特殊状況の戦略を柔軟に組み合わせ、動的な戦術実行を実現します。

## 主要機能

- **ファクトリベースの生成**: `TacticFactory`による動的戦略生成
- **多様な戦略実装**: 20種類以上の専門プランナー
- **状況適応制御**: 試合状況に応じた戦略自動選択
- **ロボット協調**: マルチロボット戦術の統合実行
- **リアルタイム計画**: 高速な戦術決定と実行

## アーキテクチャ上の役割

Craneシステムの**戦術戦略層**として、`crane_tactic_coordinator`からの指示を受けて具体的な戦術を実行し、`crane_robot_skills`に個別ロボット行動を指示します。

## プランナープラグイン一覧

### 守備系プランナー

- **GoalieSkillTactic**: ゴールキーパー専用戦略
  - ゴールライン防御
  - ボール軌道予測対応
  - クリアランス判断

- **DefenderTactic**: ディフェンダー戦略
  - エリア守備
  - マーク対応
  - インターセプト

- **TotalDefenseTactic**: 統合守備戦略

### 攻撃系プランナー

- **AttackerSkillTactic**: アタッカー戦略
  - 得点機会創出
  - パス・ドリブル選択
  - ポジション取り

- **SubAttackerSkillTactic**: アタッカー支援

### 特殊状況プランナー

- **SimpleKickOffSkillTactic**: キックオフ戦略
- **OurPenaltyKickTactic**: 自チームペナルティキック
- **TheirPenaltyKickTactic**: 敵チームペナルティキック
- **BallPlacementSkillTactic**: ボール配置戦略

### ユーティリティプランナー

- **WaiterTactic**: 待機戦略
- **SimplePlacerTactic**: エリア配置戦略
- **BallCalibrationDataCollectorTactic**: キャリブレーション用

## プランナーベースアーキテクチャ

### TacticBase基底クラス

```cpp
class TacticBase {
public:
  virtual Status calculateRobotCommand(const std::vector<RobotIdentifier>& robots) = 0;
  // ...
};
```

### プランナー生成（TacticFactory）

```cpp
// tactic_factory.cpp
auto generatePlanner(
  const std::string & tactic_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  -> TacticBase::SharedPtr
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

`crane_tactic_coordinator` の設定ファイルで名前を指定します。

```yaml
# YAML設定による自動選択
robots:
  - id: 0
    role: "goalie"
    planner: "goalie_skill"
  - id: 1
    role: "attacker"
    planner: "attacker_skill"
  - id: [2,3,4]
    role: "defender"
    planner: "defender"
```

## 最近の開発状況

### 2025年の主要変更

- **パッケージ統合**: `crane_tactics` を `crane_tactics` へ統合・改称
- **ファクトリ化**: `pluginlib` 依存からの脱却と静的マップによる管理への移行
- **スキル連携強化**: `SkillTactic` 系クラスによるスキルパッケージとの密結合化

---

**関連パッケージ**: [crane_tactic_coordinator](./crane_tactic_coordinator.md) | [crane_robot_skills](./crane_robot_skills.md) | [crane_game_analyzer](./crane_game_analyzer.md)
