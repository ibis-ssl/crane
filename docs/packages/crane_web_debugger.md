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

Material 3 Expressive ダークテーマを採用した統合デバッグポータル。

| ページ | URL | 概要 |
|--------|-----|------|
| Viewer | `/` | ES module 構成の Dock レイアウト + HUD + Replay（デフォルトページ） |
| Robot Telemetry | `/robot_telemetry.html` | ロボット別予測/実測プロット |
| Robot Test | `/robot_test.html` | インタラクティブ動作テスト |
| Annotation | `/annotation/` | タイムスタンプ付きメモ録画 PWA |

**共通アセット**: `web/shared/` に以下を配置。`ball-calibration` コンテナへは docker-compose の `volumes` で read-only マウントして共有。

- `shared/theme/m3e-theme.css` — Material 3 Expressive デザイントークン + コンポーネント
- `shared/nav/crane-nav.js` — `<crane-nav active="...">` Custom Element（全 UI で共通ナビバーを提供）
- `shared/nav/crane-nav.css` — ナビバー補助スタイル（active 強調・横スクロール）
- `shared/util/portal-links.js` — UI 間 URL 生成ヘルパ（ES module）
- `shared/` 配下はフォントと同様に `download_fonts.py` でローカル調達済み (`web/fonts/`)

## ロボット管理画面の分離

`robot_manager` は ROS 非依存の独立アプリとして [ibis-ssl/Orion_CM4](https://github.com/ibis-ssl/Orion_CM4) リポジトリへ移動しました。
`docker/dev/docker-compose.yaml` から `robot-manager` コンテナとして起動します（イメージは `ghcr.io/ibis-ssl/robot-manager:latest`）。

- URL: `http://localhost:8092`
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

- **2026年4月**: Web UI デザイン統一 — M3 Expressive テーマを `web/shared/` に共通化し、ball-calibration へ展開。全ページに共通ナビバー (`<crane-nav>`) を導入
- **2026年2月（PR #1344）**: Crane Viewer フルリデザイン（ES module 分割・Dock レイアウト・HUD・Replay）
- **2026年2月（PR #1232）**: `crane_debug_tools`から分割・独立パッケージ化
- **2026年2月（PR #1247）**: ロボット管理画面の追加
- **2026年2月（PR #1246）**: `/aggregated_svgs` レイヤー名の整理・統一
- **2026年1月（PR #1124）**: WebUI統合ポータル構築

## 関連パッケージ

- [crane_visualization_aggregator](./crane_visualization_aggregator.md) - SVGデータ統合
- [crane_visualization_interfaces](./crane_visualization_interfaces.md) - 可視化インターフェース
- [crane_msgs](./crane_msgs.md) - 配信するメッセージ定義
