# crane_debug_tools

## 概要

`crane_debug_tools`は、Craneロボットサッカーシステムの開発とデバッグを支援する最新のツールセットです。スキルのテスト実行、シナリオベースのテスト、Webベースの可視化インターフェースを提供します。

## 主要機能

### CLIスキルテスター (`cli_skill_tester.cpp`)

- コマンドラインからのスキル実行と制御
- インタラクティブモードでのスキルテスト
- 複数ロボットへの同時スキル適用
- シナリオファイルベースの自動テスト実行

### Webインターフェース

- **WebSocketサーバー** (`websocket_server.cpp`): リアルタイム双方向通信
- **Webブリッジサーバー** (`web_bridge_server.cpp`): ROS 2とWebの橋渡し
- **Webサーバー** (`simple_web_server.cpp`): HTTPサーバーとUI配信

### シナリオテスト

- JSON形式のシナリオ定義
- タイミング制御とパラメータ指定
- 繰り返しテストの自動化

## アーキテクチャ上の役割

**依存レイヤ**: 統合層（Layer 5）- 開発ツール

- `crane_robot_skills`のROS 2アクションクライアント
- `crane_msgs`のSkillExecutionアクションを利用
- 開発・デバッグフェーズでの生産性向上を目的とした補助ツール

## コンポーネント

### ノード

#### skill_tester_cli

インタラクティブなスキルテストCLIノード。

**トピック**:

- `/simple_ai/skill_execution` (Action Client) - スキル実行要求

**利用可能なスキル**:

- 基本動作: `Idle`, `Sleep`, `EmplaceRobot`
- ボール操作: `Kick`, `Receive`, `StealBall`
- ゲームロール: `Goalie`, `Attacker`, `SubAttacker`, `Marker`
- セットプレー: `GoalKick`, `SimpleKickOff`, `SingleBallPlacement`, `PenaltyKick`
- テスト用: `TestMotionPosition`, `TestMotionVelocity`, `Teleop`
- その他: `Forward`, `BallNearbyPositioner`, `SecondThreatDefender`, `FreekickSaver`

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

### 実行時依存

- `crane_session_controller` または `crane_simple_ai`（スキル実行サーバー）

## 使用方法

### CLIモードでのスキルテスト

```bash
# インタラクティブモード起動
ros2 run crane_debug_tools crane_skill_cli

# コマンド例
> list                    # 利用可能なスキル一覧
> run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0
> multi Attacker 0,1,2    # 複数ロボットに同時実行
> help                    # ヘルプ表示
> quit                    # 終了
```

### シナリオファイルの使用

**シナリオファイル例** (`test_sequence.json`):

```json
{
  "skills": [
    {
      "name": "EmplaceRobot",
      "robot_id": 0,
      "parameters": {"target_x": 1.0, "target_y": 0.0},
      "delay": 2
    },
    {
      "name": "Kick",
      "robot_id": 0,
      "parameters": {
        "target_x": 3.0,
        "target_y": 1.0,
        "kick_power": 5.0
      }
    }
  ]
}
```

**実行**:

```bash
crane_skill scenario test_sequence.json
```

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
- [crane_session_controller](./crane_session_controller.md) - スキル実行サーバー

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
