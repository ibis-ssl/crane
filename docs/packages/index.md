# Craneパッケージドキュメント ポータル

Craneを構成する全パッケージのドキュメント一覧。

## プロジェクト概要

**Crane** は ibis-ssl チームが開発するRoboCup Small Size League (SSL) 用の自律ロボットサッカーシステムです。ROS 2 Jazzy ベースで構築された、小型自律ロボットチームによるサッカー試合を制御するAIフレームワークです。

### システム特徴

- **リアルタイム制御**: SSL競技規定に準拠した高精度な制御系
- **マルチロボット協調**: RVO2アルゴリズムによる衝突回避
- **プラグインアーキテクチャ**: 戦略・スキルの拡張性
- **3D物理モデル**: ボール軌道の高精度予測

### 最近の開発活動

- **2026年1-3月**: コード品質改善キャンペーン（#1129-#1172）。全パッケージを対象にリファクタリング・バグ修正・効率化を実施
- **2025年6月**: JapanOpen2025での実戦運用完了
- **開発状況**: リファクタリングフェーズ（品質・保守性向上）

---

## 📦 パッケージ一覧

### 🔥 Core系

| パッケージ名 | 役割 |
|-------------|------|
| [crane_msgs](./crane_msgs.md) | メッセージ定義基盤 |
| [crane_world_model_publisher](./crane_world_model_publisher.md) | 世界状態推定・トラッキング |
| [crane_robot_skills](./crane_robot_skills.md) | ロボットスキルライブラリ |
| [crane_local_planner](./crane_local_planner.md) | 経路計画・衝突回避 |
| [crane_game_analyzer](./crane_game_analyzer.md) | 試合状況分析 |
| [crane_play_switcher](./crane_play_switcher.md) | プレイ自動選択 |
| [crane_sender](./crane_sender.md) | ロボットコマンド送信 |
| [crane_bringup](./crane_bringup.md) | システム起動統合 |
| [crane_description](./crane_description.md) | パラメータ管理 |
| [crane_robot_receiver](./crane_robot_receiver.md) | ロボット状態受信 |
| [crane_visualization_interfaces](./crane_visualization_interfaces.md) | 可視化インターフェース |
| [crane_speaker](./crane_speaker.md) | 音声出力システム |
| [crane_grsim_operator](./crane_grsim_operator.md) | grSim操作 |
| [crane_teleop](./crane_teleop.md) | 遠隔操作 |
| [crane_visualization_aggregator](./crane_visualization_aggregator.md) | 可視化データ統合 |

### 🎯 Session系

| パッケージ名 | 役割 |
|-------------|------|
| [crane_session_coordinator](./crane_session_coordinator.md) | 試合統括・ゲーム状態管理 |
| [crane_sessions](./crane_sessions.md) | 戦略セッションプラグイン |

### 🔧 Utility系

| パッケージ名 | 役割 |
|-------------|------|
| [crane_geometry](./crane_geometry.md) | 幾何学計算ライブラリ |
| [crane_physics](./crane_physics.md) | 物理計算・ボールモデル |
| [crane_comm](./crane_comm.md) | 通信ユーティリティ |
| [crane_msg_wrappers](./crane_msg_wrappers.md) | メッセージラッパー |
| [crane_lint_common](./crane_lint_common.md) | 共通リント設定 |
| [crane_utils](./crane_utils.md) | 共通ユーティリティ関数 |

### 📡 SSL通信系

| パッケージ名 | 役割 |
|-------------|------|
| [robocup_ssl_comm](./robocup_ssl_comm.md) | SSL通信プロトコル処理 |
| [robocup_ssl_msgs](./robocup_ssl_msgs.md) | SSL公式メッセージ定義 |

### 📚 3rdparty

| パッケージ名 | 役割 |
|-------------|------|
| [rvo2_vendor](./rvo2_vendor.md) | RVO2衝突回避アルゴリズム |
| [matplotlib_cpp_17_vendor](./matplotlib_cpp_17_vendor.md) | C++17対応matplotlib |
| [closest_point_vendor](./closest_point_vendor.md) | 最近点計算ライブラリ |

### 🛠️ 開発ツール系

| パッケージ名 | 役割 |
|-------------|------|
| [crane_bag](./crane_bag.md) | C++ rosbag解析CLIツール |
| [crane_mcap_tools](./crane_mcap_tools.md) | Python解析・SVG動画生成 |
| [crane_web_debugger](./crane_web_debugger.md) | WebSocketデバッグサーバー |
| [crane_ball_calibration_ui](./crane_ball_calibration_ui.md) | ボールモデルキャリブレーションUI |

---

## 🔗 関連リンク

- [メインドキュメント](../index.md)
- [アーキテクチャ概要](../../README.md)
- [開発ログ](../logs/)
