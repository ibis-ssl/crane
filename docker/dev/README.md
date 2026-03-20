# Docker開発環境(統合版)

このディレクトリは、シミュレーション環境(sim)と実機環境(real)を統合したDocker環境です。

## 使用方法

### スクリプトを使用した起動(推奨)

リポジトリルートから以下のコマンドを実行します。

```bash
# シミュレーション環境(デフォルト) + ssl-log-recorder自動起動
./scripts/docker-dev.sh

# 実機環境 + ssl-log-recorder自動起動
./scripts/docker-dev.sh real

# バックグラウンド起動 + debug_tools自動起動
./scripts/docker-dev.sh -d
./scripts/docker-dev.sh real -d

# debug_toolsなしで起動
./scripts/docker-dev.sh --no-debug

# 停止(debug_toolsとssl-log-recorderも自動停止)
./scripts/docker-dev.sh down
```

**注1**: `up` 実行時（フォアグラウンド/バックグラウンドの両方）に
`ssl-log-recorder` (`robocupssl/ssl-log-recorder:latest`) が自動的に起動し、
ログはリポジトリルートに保存されます。

**注2**: `ssl-log-recorder` は `./scripts/docker-dev.sh down` 実行時に停止します。
`up` を Ctrl+C で終了した場合は recorder は継続起動します。

**注3**: バックグラウンド起動(`-d`)時、debug_tools
(crane_websocket_server) が自動的に起動します。

- HTTP Server: <http://localhost:8090>
- WebSocket: ws://localhost:8091
- `--no-debug` オプションで無効化可能

### debug_tools の手動操作

debug_tools を個別に操作したい場合は、以下のスクリプトを使用できます。

```bash
# debug_tools を起動
./scripts/start-debug-tools.sh

# debug_tools を停止
./scripts/stop-debug-tools.sh
```

### Docker Composeコマンドでの直接起動

```bash
# シミュレーション環境
docker compose -f docker/dev/docker-compose.yaml --profile sim up

# 実機環境
VISION_PORT=10006 docker compose -f docker/dev/docker-compose.yaml up

# 停止
docker compose -f docker/dev/docker-compose.yaml down
```

## sim/real の差分

| 項目 | sim | real |
|------|-----|------|
| Vision ポート | 10020 | 10006 |
| ssl-status-board | 有効 | 無効 |

環境変数 `VISION_PORT` とDocker Composeの `profiles` 機能を使用して切り替えています。

## サービス一覧

- **ssl-game-controller**: RoboCup SSLのゲームコントローラー
- **ssl-vision-client**: SSL Vision クライアント
- **ssl-status-board**: ステータスボード(simのみ)
- **autoref-tigers**: Tigers Mannheimのオートレフェリー
- **voicevox**: 音声合成エンジン
- **debug_tools**: WebSocketベースのデバッグツール(docker-dev.shで自動起動)

## 設定ファイル

- `docker-compose.yaml`: サービス定義
- `.env`: 環境変数(Visionポートなどのデフォルト設定)
- `config/`: ゲームコントローラー設定ファイル
