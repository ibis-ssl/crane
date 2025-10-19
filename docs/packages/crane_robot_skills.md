# crane_robot_skills

## 概要

**crane_robot_skills**パッケージは、SSL（RoboCup Small Size League）自律ロボットサッカーにおける個別ロボットの行動スキルライブラリです。統一されたインターフェースを通じて高次戦略プランナーから呼び出し可能な、再利用可能なスキルを提供します。シンプルなスキルから複雑な状態機械ベースのスキルまで、25以上の戦術的行動を実装し、Craneシステムの行動レイヤーを構成します。

## 主要機能

- **スキルベースアーキテクチャ**: 25種類以上の個別ロボット行動スキル
- **統一インターフェース**: SkillBaseクラスによる一貫したスキル実行API
- **戦術的行動**: 攻撃・守備・特殊状況に対応した高度な行動パターン
- **リアルタイム実行**: 制御周期での動的な行動決定と実行

## アーキテクチャ上の役割

Craneシステムの**行動実行層**として、戦略プランナーからの高レベル指示を具体的なロボット制御コマンドに変換します。SkillBaseクラスを基底とするプラグイン的アーキテクチャにより、新しいスキルの追加と既存スキルの組み合わせが容易です。

## スキル一覧

### 攻撃系スキル

- **AttackerSkill**: メインアタッカーの総合攻撃行動
- **SubAttackerSkill**: サブアタッカーの支援攻撃行動
- **KickSkill**: 精密キック実行
- **ReceiveSkill**: パス受け取り
- **StealBallSkill**: ボール奪取

### 守備系スキル

- **GoalieSkill**: ゴールキーパー専用行動
- **SecondThreatDefenderSkill**: セカンドディフェンダー
- **FreekickSaverSkill**: フリーキック対応守備

### 特殊状況スキル

- **KickoffSupportSkill**: キックオフ時支援
- **SimpleKickoffSkill**: 基本キックオフ
- **PenaltyKickSkill**: ペナルティキック実行
- **GoalKickSkill**: ゴールキック実行
- **SingleBallPlacementSkill**: ボール配置

### ポジショニング系スキル

- **ForwardSkill**: フォワードポジション取り
- **MarkerSkill**: マーク行動
- **BallNearbyPositionerSkill**: ボール近傍位置取り
- **EmplaceRobotSkill**: 指定位置配置

### ユーティリティスキル

- **IdleSkill**: 待機状態
- **SleepSkill**: 休止状態
- **TeleopSkill**: 手動操縦
- **TestMotionSkill**: テスト動作
- **RobotCommandAsSkill**: 直接コマンド実行

## スキルベースアーキテクチャ

### SkillBase基底クラス

```cpp
class SkillBase {
public:
  virtual Status update() = 0;
  virtual Status getStatus() const = 0;
  virtual void reset() = 0;

protected:
  WorldModelWrapper::SharedPtr world_model;
  GameAnalysisWrapper::SharedPtr game_analysis;
};
```

### スキル実行ステータス

```cpp
enum class Status {
  RUNNING,     // 実行中
  SUCCESS,     // 成功完了
  FAILURE,     // 失敗
  NEED_REPLAN  // 再計画要求
};
```

### スキル合成パターン

```cpp
// 複合スキルの例：アタッカー行動
Status AttackerSkill::run(RobotCommandWrapperPosition & command) {
  if (shouldKick()) {
    return kick_skill_->run(command);
  } else if (shouldReceive()) {
    return receive_skill_->run(command);
  } else {
    return positionForAttack(command);
  }
}
```

## 高度な実装特徴

### 状況適応制御

- **動的行動切り替え**: 試合状況に応じたリアルタイム行動変更
- **予測制御**: ボール・敵ロボットの未来位置を考慮した行動計画
- **学習的調整**: 試合中のパフォーマンスフィードバックによる行動最適化

### 物理制約考慮

- **ロボット動力学**: 加速度・角速度制限を考慮した実現可能な制御
- **衝突回避**: 他ロボットとの衝突を避ける安全な経路生成
- **キック力学**: ボール物理とロボット機構を考慮した最適キック

## 依存関係

### コア依存

- **crane_geometry**: 幾何学計算ライブラリ
- **crane_physics**: 物理計算・ボールモデル
- **crane_game_analyzer**: 試合状況分析
- **crane_msg_wrappers**: メッセージ変換ユーティリティ
- **crane_msgs**: システムメッセージ定義

### システム依存

- **boost**: 高性能C++ライブラリ
- **magic_enum**: enum反射機能
- **rclcpp_components**: ROS 2コンポーネント機能

## 使用方法

### スキル単体実行

```cpp
#include "crane_robot_skills/attacker.hpp"

auto attacker = std::make_shared<AttackerSkill>(world_model, game_analysis);
RobotCommandWrapperPosition command;

// スキル実行
Status status = attacker->run(command);
if (status == Status::SUCCESS) {
  // コマンド送信
  sendCommand(command);
}
```

### プランナーからの利用

```cpp
// プランナープラグインでの使用例
void AttackerSkillPlanner::plan() {
  for (auto robot_id : assigned_robots) {
    auto & skill = robot_skills_[robot_id];
    RobotCommandWrapperPosition cmd;

    if (skill->run(cmd) != Status::FAILURE) {
      robot_commands_->addCommand(robot_id, cmd);
    }
  }
}
```

### カスタムスキル作成

```cpp
class CustomSkill : public SkillBase {
public:
  Status run(RobotCommandWrapperPosition & command) override {
    // カスタム行動実装
    return Status::RUNNING;
  }

  void reset() override {
    // 状態リセット
  }
};
```

## 最近の開発状況

### 2025年の主要変更

- **ポゼッション維持強化**: 抑制系スキルにボール保護モードを追加
- **セットプレー拡充**: JapanOpen向けリスタートスキル群を統合
- **パフォーマンス最適化**: 計算パスを整理し実行周期のばらつきを抑制
- **状況適応改良**: Planner連携用のコンテキスト引数を標準化

### 開発活発度

🟡 **中活動**: 攻撃系・ゴールキーパー系スキルのリファインが継続し、プランナーパッケージとの連携改善を中心にアップデートが行われている。

### 今後の発展方向

- **AI統合**: 機械学習による行動最適化
- **チーム連携**: より高度なマルチロボット協調行動
- **適応制御**: 対戦相手に応じた動的戦術調整

## パフォーマンス特性

### 実行特性

- **応答性**: <10ms（スキル決定→コマンド生成）
- **精度**: 位置制御±2cm、角度制御±2度
- **成功率**: 基本スキル>95%、複合スキル>85%

### リソース使用量

- **CPU使用率**: 1-3%（ロボット1台あたり）
- **メモリ使用量**: 10-50MB（スキルセット）

---

**関連パッケージ**: [crane_planner_plugins](./crane_planner_plugins.md) | [crane_local_planner](./crane_local_planner.md) | [crane_game_analyzer](./crane_game_analyzer.md)
