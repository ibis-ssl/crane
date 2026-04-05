# crane_web_debugger

## 概要

`crane_web_debugger`は、WebSocketサーバーと統合HTTPサーバーを提供するリアルタイムデバッグツールです。ROS 2トピックをブラウザから監視・制御するインターフェースに加え、ボールモデルキャリブレーションUIも同一ポートで提供します。

## 主要機能

- **WebSocketサーバー**: リアルタイム双方向通信（Boost.Asio実装）
- **統合HTTPサーバー**: FastAPIベースでデバッグUIとキャリブレーションUIを同一配信
- **Webフロントエンド**: ブラウザベースのデバッグUI・ロボット管理画面・キャリブレーション画面
- **ROS 2ブリッジ**: 以下のトピックをWebSocket経由で配信
  - `/world_model`
  - `/robot_commands`
  - `/robot_feedback_array`
  - `/play_situation`
  - `/diagnostics`
  - SVGビジュアライゼーション（`/aggregated_svgs`）
  - ヒューマンアノテーション

## アーキテクチャ上の役割

**依存レイヤ**: 統合層（Layer 5）- 開発ツール

開発・デバッグフェーズでの生産性向上を目的とした補助ツール。実際の試合では使用しない。

## コンポーネント

### ノード

#### websocket_server

ROS 2とブラウザを接続するブリッジノード。

- WebSocketハンドシェイク（SHA-1/Base64実装）
- 複数クライアントの同時接続管理
- ROS 2トピックのJSON変換・配信

### Webフロントエンド

- SVGビジュアライゼーション表示
- デバッグUI統合ポータル
- Ball Calibration UI（`/ball-calibration/`）

### Ball Calibration API

- `POST /ball-calibration/api/load`
- `GET /ball-calibration/api/trajectories`
- `GET /ball-calibration/api/trajectory/{event_id}`
- `POST /ball-calibration/api/optimize`
- `POST /ball-calibration/api/predict`
- `POST /ball-calibration/api/export`

## ロボット管理画面の分離

`robot_manager` は ROS 非依存の独立アプリとして `docker/dev/robot-manager/`
へ分離されました。`docker/dev/docker-compose.yaml` から
`robot-manager` コンテナとして起動します。

- URL: `http://localhost:8090`
- 機能: Start/Stop/Status（Pi HTTP API経由）
- ROSトピック由来の表示（world_model/control_targets等）は含みません

## 依存関係

- `crane_msgs`, `crane_visualization_interfaces`
- `diagnostic_msgs`, `std_msgs`, `rclcpp`
- `ament_index_cpp`
- `libboost-system-dev`, `libssl-dev`, `nlohmann-json-dev`
- `python3-fastapi`, `python3-uvicorn`, `python3-numpy`, `python3-pydantic`, `python3-yaml`

## 使用方法

```bash
# WebSocketサーバーを起動
ros2 run crane_web_debugger crane_websocket_server

# ブラウザでアクセス
# http://localhost:8090/
# http://localhost:8090/ball-calibration/
```

### Docker環境での自動起動

```bash
# Docker起動時にWebSocketサーバーが自動起動されます
./scripts/docker-dev.sh -d
```

## 最近の開発状況

- **2026年2月（PR #1232）**: `crane_debug_tools`から分割・独立パッケージ化
- **2026年2月（PR #1247）**: ロボット管理画面の追加
- **2026年2月（PR #1246）**: `/aggregated_svgs` レイヤー名の整理・統一
- **2026年1月（PR #1124）**: WebUI統合ポータル構築

## 関連パッケージ

- [crane_visualization_aggregator](./crane_visualization_aggregator.md) - SVGデータ統合
- [crane_visualization_interfaces](./crane_visualization_interfaces.md) - 可視化インターフェース
- [crane_msgs](./crane_msgs.md) - 配信するメッセージ定義
