# Crane Debug Tools

Craneロボットサッカーシステム用のデバッグ・テストツール。

## インストール

```bash
# ビルド
colcon build --packages-select crane_debug_tools

# ワークスペースを読み込み
source install/local_setup.bash
```

## 基本的な使い方

### CLI コマンド

```bash
# 利用可能なスキルをリスト表示
crane_skill list

# スキルを実行(パラメータはkey:value形式)
crane_skill run Kick 0 target_x:1.0 target_y:2.0 kick_power:5.0

# 複数ロボットで実行
crane_skill multi Attacker 0,1,2

# シナリオファイルを実行
crane_skill scenario test_sequence.json
```

### Web インターフェース

#### 推奨: Docker環境での自動起動

```bash
# Docker起動時にWebSocketサーバーが自動起動されます
./scripts/docker-dev.sh -d

# ブラウザでアクセス
http://localhost:8090/standalone.html
```

WebSocketサーバー(crane_websocket_server)はDocker起動スクリプト(`./scripts/docker-dev.sh`)により自動的に起動されます。

#### 手動起動(オプション)

```bash
# WebSocketサーバーを手動で起動
./scripts/start-debug-tools.sh

# 停止
./scripts/stop-debug-tools.sh
```

#### その他のデバッグツール

```bash
# CLI/Webインターフェースを起動
ros2 launch crane_debug_tools debug_tools.launch.py enable_web:=true

# CLIツールのみ
ros2 launch crane_debug_tools debug_tools.launch.py enable_cli:=true
```

## シナリオファイル形式

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
      "parameters": {"target_x": 3.0, "target_y": 1.0, "kick_power": 5.0}
    }
  ]
}
```

## 主要スキル一覧

- `Idle`, `Sleep`, `EmplaceRobot` - 基本動作
- `Kick`, `Receive` - ボール操作
- `Goalie`, `Attacker`, `SubAttacker` - ゲームロール
- `GoalKick`, `SimpleKickOff`, `SingleBallPlacement` - セットプレー
- `Teleop` - テスト用

## トラブルシューティング

- **コマンドが見つからない**: `source install/local_setup.bash` を実行
- **アクションサーバーに接続できない**: craneシステムが起動しているか確認
- **パラメータエラー**: `key:value` 形式を使用(`key=value` ではない)
