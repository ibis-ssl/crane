# Crane Documentation

**Crane**は、ibis-ssl チームが開発するRoboCup Small Size League (SSL) 用の自律ロボットサッカーシステムです。ROS 2 Jazzy ベースで構築された、小型自律ロボットチームによるサッカー試合を制御するAIフレームワークです。

## 🚀 はじめに

### クイックスタート

- **[環境構築](./setup.md)** - ROS 2 Jazzy環境のセットアップガイド
- **[開発進捗](./progress.md)** - 2025年JapanOpen後の開発状況と機能概要
- **[Docker環境](./docker.md)** - シミュレーション環境の構築

### 開発者向け

- **[開発ツール](./tools.md)** - コード品質ツールと便利なコマンド
- **[座標系仕様](./coordinates.md)** - フィールド・ロボット座標系の詳細定義

## 📦 システムアーキテクチャ

### 🔥 基盤コンポーネント

#### [crane_msgs](./packages/crane_msgs.md)

メッセージ定義基盤 - システム全体の通信インフラを支える中核パッケージ

- Analysis/Control/WorldModel 3層メッセージ構造
- 型安全なROS 2通信の実現

#### [crane_world_model_publisher](./packages/crane_world_model_publisher.md)  

世界状態推定・トラッキング - 高精度な知覚システム

- EKFベース高精度ボールトラッキング
- 3D物理モデル（重力・空気抵抗対応）
- 自動キャリブレーション機能
- リアルタイム60Hz更新

#### [crane_robot_skills](./packages/crane_robot_skills.md)

ロボットスキルライブラリ - 25以上の戦術的行動を実装

- 攻撃・守備・特殊状況スキルの包括的カタログ
- 状態機械とパラメータシステム
- 可視化統合対応

#### [crane_session_controller](./packages/crane_session_controller.md)

試合統括・ゲーム状態管理 - 最上位制御レイヤー

- セッションベースロボット役割管理
- YAML駆動戦術設定システム
- 動的プランナープラグイン管理

#### [crane_geometry](./packages/crane_geometry.md)

数学的基盤パッケージ - 幾何学計算とBoost.Geometry統合

- カスタム2D/3Dベクトルクラス（Eigen風API）
- 幾何学形状と座標変換の包括的実装
- 型安全で効率的な幾何学操作基盤

#### [crane_physics](./packages/crane_physics.md)

物理シミュレーションパッケージ - Header-only高性能ライブラリ

- 3D状態ベース ボール物理モデル（STOPPED/ROLLING/FLYING）
- 台形運動プロファイル・PID制御・ハンガリアンアルゴリズム
- リアルタイム（60-100Hz）制御ループ最適化設計

## 🎯 専門技術ドキュメント

### ロボット制御

- **[スキルシステム](./skill.md)** - 個別ロボット行動の詳細実装ガイド
- **[攻撃戦術](./attacker.md)** - Attackerスキルの状態遷移と実装
- **[守備システム](./defense.md)** - ゴールキーパーとディフェンダーの戦術

### システム技術  

- **[ボールトラッキング](./ball_tracking_system.md)** - EKFベース状態推定の技術仕様
- **[ボールモデルキャリブレーション](./ball_model_calibration_guide.md)** - 物理パラメータ自動最適化
- **[可視化システム](./visualizer.md)** - SVGベース可視化APIの使用ガイド
- **[ネットワーク設定](./network.md)** - SSL通信とマルチキャスト設定

### 試合運用

- **[試合チェックリスト](./match.md)** - 試合前確認事項
- **[SSL Vision設定](./vision.md)** - カメラキャリブレーションとパターン認識
- **[grSim環境](./grSim.md)** - シミュレーション環境の使用方法

## 📚 パッケージドキュメント

**32パッケージの詳細仕様** - [📦 パッケージ一覧](./packages/index.md)

現在の完成状況:

- ✅ **全パッケージ完了**: 32/32 パッケージ (100%)
- ✅ **基盤パッケージ**: 6/6 完了 (crane_msgs, crane_world_model_publisher, crane_robot_skills, crane_session_controller, crane_geometry, crane_physics)

## 🔗 外部リソース

- **[ibis-ssl ドキュメント](https://ibis-ssl.github.io/ibis_documentation/)** - チーム公式ドキュメント
- **[GitHub リポジトリ](https://github.com/ibis-ssl/crane)** - 最新ソースコード
- **[RoboCup SSL公式](https://ssl.robocup.org/)** - 競技規則と技術仕様

## 🛠️ 最新の開発状況

### 最新の開発成果

- **2025年6月**: JapanOpen2025での実戦投入・競技実績を達成
- **2025年前半**: 全32パッケージのドキュメント完成・システム統合完了
- **2024年末**: ボールフィルタ実装・3Dボールモデル拡充・Vector3d導入完了

### 技術的特徴

- **リアルタイム制御**: 60Hz制御ループでの高精度ロボット協調
- **3D物理モデル**: 重力・空気抵抗を考慮した予測システム
- **プラグインアーキテクチャ**: 戦略・スキルの動的拡張
- **YAML設定駆動**: コード変更なしでの戦術調整

---

**📧 質問・フィードバック**: [GitHub Issues](https://github.com/ibis-ssl/crane/issues)で承ります。
