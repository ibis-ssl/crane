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

- **Attacker**: メインアタッカーの総合攻撃行動
- **SubAttacker**: サブアタッカーの支援攻撃行動
- **Kick**: 精密キック実行
- **CenterStopKick**: フィールド中心停止キック実行
- **Receive**: パス受け取り（敵割り込み回避機能付き）
  - パスライン上の敵ロボットを検出（0.3m閾値）
  - Slack Timeベースの代替受け取り位置探索
  - 自動的な最適位置への移動

### 守備系スキル

- **Goalie**: ゴールキーパー専用行動
- **SecondThreatDefender**: セカンドディフェンダー
- **Marker**: マーク行動

### 特殊状況スキル

- **SimpleKickoff**: 基本キックオフ
- **PenaltyKick**: ペナルティキック実行
- **GoalKick**: ゴールキック実行
- **SingleBallPlacement**: ボール配置

### ポジショニング系スキル

- **BallNearbyPositioner**: ボール近傍位置取り
- **EmplaceRobot**: 指定位置配置
- **Forward**: 前線ポジショニング（パスコース確保・敵回避）

### ユーティリティスキル

- **Idle**: 待機状態
- **Sleep**: 休止状態
- **Teleop**: 手動操縦
- **BallCalibrationDataCollector**: ボールモデル学習用データ収集

## スキルベースアーキテクチャ

### SkillBase基底クラス

```cpp
namespace crane::skills {

enum class Status {
  SUCCESS,
  FAILURE,
  RUNNING,
};

class SkillInterface {
public:
  // パラメータを指定して実行
  virtual Status run(
    std::optional<std::unordered_map<std::string, ParameterType>> parameters_opt = std::nullopt) = 0;

  // 生成されたコマンドの取得
  virtual crane_msgs::msg::RobotCommand getRobotCommand() = 0;

  // パラメータ操作
  void setParameter(const std::string & key, const T & value);
  template <class T> auto getParameter(const std::string & key) const;
};

class SkillBase : public SkillInterface {
public:
  virtual Status update() = 0;

protected:
  // コマンド操作用ラッパー
  std::shared_ptr<PositionCommandWrapper> command;
};

} // namespace crane::skills
```

### スキル合成パターン

```cpp
// 複合スキルの例：アタッカー行動（Attacker.cppより概念的抜粋）
Status Attacker::update() {
  // 状態機械による制御
  // ...
  if (current_state == AttackerState::KICK) {
      // 下位スキル(Kick)の実行
      // コマンドラッパーを共有して実行することで、下位スキルのコマンドが反映される
      return kick_skill.run();
  }
  // ...
}
```

## 依存関係

### コア依存

- **crane_geometry**: 幾何学計算ライブラリ
- **crane_physics**: 物理計算・ボールモデル
- **crane_msg_wrappers**: メッセージ変換ユーティリティ
- **crane_msgs**: システムメッセージ定義

### システム依存

- **boost**: 高性能C++ライブラリ
- **magic_enum**: enum反射機能
- **rclcpp**: ROS 2クライアントライブラリ

## 使用方法

### スキル単体実行

```cpp
#include "crane_robot_skills/attacker.hpp"

// スキルインスタンス化
auto attacker = std::make_shared<crane::skills::Attacker>(robot_id, world_model);

// パラメータ設定（必要な場合）
attacker->setParameter("target", Point(1.0, 0.0));

// スキル実行
auto status = attacker->run();

// コマンド取得
auto command_msg = attacker->getRobotCommand();
```

### タクティックからの利用

```cpp
// タクティック内での使用例
void MyTactic::calculate_robot_command(const RobotInfo::SharedPtr & robot) {
    // スキルの取得または生成
    auto skill = get_skill<Attacker>(robot->id);

    // スキル実行
    skill->run();

    // コマンドをシステムに登録
    robot_commands->setCommand(skill->getRobotCommand());
}
```

### カスタムスキル作成

```cpp
class CustomSkill : public SkillBase {
public:
  explicit CustomSkill(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
    : SkillBase("CustomSkill", id, wm) {}

  Status update() override {
    // カスタム行動実装
    commander()->setTargetPosition(getParameter<Point>("target"));
    return Status::RUNNING;
  }
};
```

## 最近の開発状況

### 2025年の主要変更

- **ポゼッション維持強化**: 抑制系スキルにボール保護モードを追加
- **セットプレー拡充**: JapanOpen向けリスタートスキル群を統合
- **パフォーマンス最適化**: 計算パスを整理し実行周期のばらつきを抑制
- **状況適応改良**: Tactic連携用のコンテキスト引数を標準化

### 開発活発度

🟡 **中活動**: 攻撃系・ゴールキーパー系スキルのリファインが継続し、タクティックパッケージとの連携改善を中心にアップデートが行われている。

---

**関連パッケージ**: [crane_local_planner](./crane_local_planner.md) | [crane_game_analyzer](./crane_game_analyzer.md)
