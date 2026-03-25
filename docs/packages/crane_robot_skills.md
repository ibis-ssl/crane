# crane_robot_skills

## 概要

**crane_robot_skills**パッケージは、SSL（RoboCup Small Size League）自律ロボットサッカーにおける個別ロボットの行動スキルライブラリです。統一されたインターフェースを通じて高次戦略プランナーから呼び出し可能な、再利用可能なスキルを提供します。シンプルなスキルから複雑な状態機械ベースのスキルまで、多様な戦術的行動を実装し、Craneシステムの行動レイヤーを構成します。

## 主要機能

- **スキルベースアーキテクチャ**: 多様な個別ロボット行動スキル
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

`namespace crane::skills` に定義。主な要素：

- **`Status`列挙型**: `SUCCESS` / `FAILURE` / `RUNNING`
- **`SkillInterface`**: 全スキルの共通インターフェース（`run()`, `getRobotCommand()`, `setParameter()`, `getParameter()`）
- **`SkillBase`**: シンプルスキル用基底クラス。`update()`を実装する
- **`SkillBaseWithState`**: 状態機械を内蔵した基底クラス。`addStateFunction()` + `addTransition()`で状態遷移を定義する

コマンドラッパーへのアクセスは `commander()` メソッド経由で行う。

### スキル合成パターン

`SkillBaseWithState` を継承したスキル（例: `Attacker`）は、内部で下位スキル（例: `Kick`）を保持し、状態に応じて下位スキルの `run()` を呼び出すことで複合的な行動を実現する。

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

スキルは `robot_id` と `world_model` を引数にインスタンス化し、`setParameter()` でパラメータを設定後、`run()` で実行する。生成されたコマンドは `getRobotCommand()` で取得する。

カスタムスキルは `SkillBase`（シンプルな行動）または `SkillBaseWithState`（状態遷移が必要な行動）を継承して実装する。実装例は既存スキルのソースコードを参照。

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
