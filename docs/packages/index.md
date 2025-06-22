# Craneパッケージドキュメント ポータル

> **📋 Claude Code並列作業対応**  
> このページは複数のClaude Codeセッションでの並列ドキュメント作成を支援します。各セッションで担当するパッケージ群を選択し、作業状況を更新してください。

## プロジェクト概要

**Crane** は ibis-ssl チームが開発するRoboCup Small Size League (SSL) 用の自律ロボットサッカーシステムです。ROS 2 Jazzy ベースで構築された、小型自律ロボットチームによるサッカー試合を制御するAIフレームワークです。

### システム特徴

- **リアルタイム制御**: SSL競技規定に準拠した高精度な制御系
- **マルチロボット協調**: RVO2アルゴリズムによる衝突回避
- **プラグインアーキテクチャ**: 戦略・スキルの拡張性
- **3D物理モデル**: ボール軌道の高精度予測

### 最近の開発活動

- **2024年12月**: ボールフィルタ実装(#881)・ボールモデル拡充(#872)
- **2024年11月**: Vector3d導入(#880)・Claude Code開発ドキュメント追加(#879)
- **開発状況**: 非常に活発（月20+コミット）

---

## 📦 パッケージ一覧・作業状況

### 🔥 Core系パッケージ群 (14/14) - 高優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [crane_msgs](./crane_msgs.md) | メッセージ定義基盤 | 🟢 安定 | ❌ 未作成 | - |
| [crane_world_model_publisher](./crane_world_model_publisher.md) | 世界状態推定・トラッキング | 🔴 高活動 | ❌ 未作成 | - |
| [crane_robot_skills](./crane_robot_skills.md) | ロボットスキルライブラリ | 🟡 中活動 | ❌ 未作成 | - |
| [crane_local_planner](./crane_local_planner.md) | 経路計画・衝突回避 | 🔴 高活動 | ❌ 未作成 | - |
| [crane_game_analyzer](./crane_game_analyzer.md) | 試合状況分析 | 🟡 中活動 | ❌ 未作成 | - |
| [crane_play_switcher](./crane_play_switcher.md) | プレイ自動選択 | 🟡 中活動 | ❌ 未作成 | - |
| [crane_sender](./crane_sender.md) | ロボットコマンド送信 | 🟢 安定 | ❌ 未作成 | - |
| [crane_bringup](./crane_bringup.md) | システム起動統合 | 🟢 安定 | ❌ 未作成 | - |
| [crane_description](./crane_description.md) | パラメータ管理 | 🟢 安定 | ❌ 未作成 | - |
| [crane_robot_receiver](./crane_robot_receiver.md) | ロボット状態受信 | 🟢 安定 | ❌ 未作成 | - |
| [crane_visualization_interfaces](./crane_visualization_interfaces.md) | 可視化インターフェース | 🟢 安定 | ❌ 未作成 | - |
| [crane_gui](./crane_gui.md) | GUI（開発中止） | ⚫ 無効 | ❌ 未作成 | - |
| [crane_simple_ai](./crane_simple_ai.md) | 簡易AI制御 | 🟢 安定 | ❌ 未作成 | - |
| [crane_speaker](./crane_speaker.md) | 音声出力システム | 🟢 安定 | ❌ 未作成 | - |

### 🎯 Session系パッケージ群 (2/2) - 高優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [crane_session_controller](./crane_session_controller.md) | 試合統括・ゲーム状態管理 | 🟡 中活動 | ❌ 未作成 | - |
| [crane_planner_plugins](./crane_planner_plugins.md) | 戦略プランナープラグイン | 🔴 高活動 | ❌ 未作成 | - |

### 🔧 Utility系パッケージ群 (10/10) - 中優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [crane_geometry](./crane_geometry.md) | 幾何学計算ライブラリ | 🔴 高活動 | ❌ 未作成 | - |
| [crane_physics](./crane_physics.md) | 物理計算・ボールモデル | 🔴 高活動 | ❌ 未作成 | - |
| [crane_comm](./crane_comm.md) | 通信ユーティリティ | 🟡 中活動 | ❌ 未作成 | - |
| [crane_msg_wrappers](./crane_msg_wrappers.md) | メッセージラッパー | 🟢 安定 | ❌ 未作成 | - |
| [crane_clock_publisher](./crane_clock_publisher.md) | システム時刻同期 | 🟢 安定 | ❌ 未作成 | - |
| [crane_grsim_operator](./crane_grsim_operator.md) | grSim操作 | 🟢 安定 | ❌ 未作成 | - |
| [crane_lint_common](./crane_lint_common.md) | 共通リント設定 | 🟢 安定 | ❌ 未作成 | - |
| [crane_teleop](./crane_teleop.md) | 遠隔操作 | 🟢 安定 | ❌ 未作成 | - |
| [crane_visualization_aggregator](./crane_visualization_aggregator.md) | 可視化データ統合 | 🟢 安定 | ❌ 未作成 | - |
| ~~[crane_basics](./crane_basics.md)~~ | ~~基礎ユーティリティライブラリ~~ | ⚫ 解体済 | ⚠️ 非推奨 | - |

### 📡 SSL通信系パッケージ群 (3/3) - 中優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [robocup_ssl_comm](./robocup_ssl_comm.md) | SSL通信プロトコル処理 | 🟢 安定 | ❌ 未作成 | - |
| [robocup_ssl_msgs](./robocup_ssl_msgs.md) | SSL公式メッセージ定義 | 🟢 安定 | ❌ 未作成 | - |
| [consai_visualizer](./consai_visualizer.md) | SSL可視化GUI | 🟢 安定 | ❌ 未作成 | - |

### 📚 3rdparty系パッケージ群 (3/3) - 低優先度

| パッケージ名 | 役割 | 開発活発度 | ドキュメント状況 | 担当セッション |
|-------------|------|-----------|----------------|---------------|
| [rvo2_vendor](./rvo2_vendor.md) | RVO2衝突回避アルゴリズム | 🟢 安定 | ❌ 未作成 | - |
| [matplotlib_cpp_17_vendor](./matplotlib_cpp_17_vendor.md) | C++17対応matplotlib | 🟢 安定 | ❌ 未作成 | - |
| [closest_point_vendor](./closest_point_vendor.md) | 最近点計算ライブラリ | 🟢 安定 | ❌ 未作成 | - |

---

## 🚀 並列作業ガイドライン

### セッション分担推奨

1. **セッション1**: Core系パッケージ群 (crane_msgs, crane_world_model_publisher, crane_robot_skills等)
2. **セッション2**: Session系 + 高活動Utility系 (crane_session_controller, crane_planner_plugins, crane_geometry, crane_physics)
3. **セッション3**: Utility系 + SSL通信系 (crane_comm, crane_msg_wrappers以下 + robocup_ssl_comm等)
4. **セッション4**: 3rdparty系 + 最終整理 (vendor系 + リンク設置)

### 作業手順

1. 担当パッケージ群を上記表の「担当セッション」欄に記入
2. 各パッケージのドキュメント作成
3. 「ドキュメント状況」を✅作成済みに更新
4. シンボリックリンク設置（最終工程）

### ドキュメントテンプレート

```markdown
# パッケージ名

## 概要
[パッケージの目的と概要]

## 主要機能
- 機能1
- 機能2

## アーキテクチャ上の役割
[システム内での位置づけ]

## コンポーネント
### ノード
- ノード名: 機能説明

### ライブラリ
- ライブラリ名: 機能説明

## 依存関係
- パッケージ依存
- システム依存

## 使用方法
[基本的な使用例]

## 最近の開発状況
[2024年の主要な変更と活動度]
```

---

## 📊 進捗サマリー

- **全体進捗**: 0/32 パッケージ (0%)
- **Core系**: 0/14 完了
- **Session系**: 0/2 完了  
- **Utility系**: 0/10 完了
- **SSL通信系**: 0/3 完了
- **3rdparty系**: 0/3 完了

**最終更新**: 2024-06-20

---

## 🔗 関連リンク

- [メインドキュメント](../index.md)
- [アーキテクチャ概要](../README.md)
- [開発ログ](../logs/)
- [Claude Code開発ドキュメント](../CLAUDE.md)

---

## 📝 注意事項

1. **並列作業時**: 担当セッション欄を必ず更新してください
2. **ドキュメント品質**: 技術的正確性と日本語統一性を保持
3. **リンク整合性**: パッケージ間参照の正確性を確認
4. **定期更新**: 開発進捗に応じてドキュメント更新
