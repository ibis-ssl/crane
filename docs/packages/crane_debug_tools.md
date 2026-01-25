# crane_debug_tools

## 概要

`crane_debug_tools`は、Craneロボットサッカーシステムの開発とデバッグを支援する最新のツールセットです。スキルのテスト実行、シナリオベースのテスト、Webベースの可視化インターフェースを提供します。

## 主要機能

### Webインターフェース

- **WebSocketサーバー** (`websocket_server.cpp`): リアルタイム双方向通信
- **Webブリッジサーバー** (`web_bridge_server.cpp`): ROS 2とWebの橋渡し
- **Webサーバー** (`simple_web_server.cpp`): HTTPサーバーとUI配信

## アーキテクチャ上の役割

**依存レイヤ**: 統合層（Layer 5）- 開発ツール

- 開発・デバッグフェーズでの生産性向上を目的とした補助ツール

## コンポーネント

### ノード

#### debug_web_server

Web UIとROS 2を接続するブリッジノード。

**機能**:

- WebSocket経由のリアルタイムデータ配信
- ブラウザからのスキル実行制御
- 可視化データのストリーミング

## 依存関係

### ビルド依存

- `ament_cmake_auto`
- `crane_msg_wrappers`
- `crane_msgs`
- `crane_robot_skills`
- `crane_visualization_interfaces`
- `rclcpp`, `rclcpp_action`
- `nlohmann-json-dev`
- `libboost-system-dev`, `libssl-dev`, `libgoogle-glog-dev`

## 使用方法

### Webインターフェース

```bash
# Webサーバー有効化で起動
ros2 launch crane_debug_tools debug_tools.launch.py enable_web:=true

# ブラウザでアクセス
http://localhost:8080/standalone.html
```

## 最近の開発状況

- **2025年前半**: WebSocket/HTTP統合によるリアルタイム可視化強化
- **2025年JapanOpen**: 大会時のデバッグツールとして実戦運用
- **設計方針**: 開発効率向上のための実用的なツール群として継続拡充

## 関連パッケージ

- [crane_robot_skills](./crane_robot_skills.md) - テスト対象のスキルライブラリ
- [crane_msgs](./crane_msgs.md) - SkillExecutionアクションメッセージ
- [crane_session_coordinator](./crane_session_coordinator.md) - スキル実行サーバー

## トラブルシューティング

### アクションサーバーに接続できない

**原因**: Craneシステムが起動していない

**解決策**:

```bash
ros2 launch crane_bringup crane.launch.xml sim:=true
```

### コマンドが見つからない

**原因**: 環境変数が読み込まれていない

**解決策**:

```bash
source install/local_setup.bash
```

### パラメータエラー

**原因**: 不正なパラメータ形式

**正しい形式**: `key:value` （`key=value` ではない）

## 備考

- 本パッケージは開発・テスト専用であり、実際の試合では使用しない
- WebインターフェースはデバッグUIとして便利だが、パフォーマンスへの影響に注意
- シナリオテストは回帰テストの自動化に有効
