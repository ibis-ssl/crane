# Docker開発環境(統合版)

このディレクトリは、シミュレーション環境(sim)と実機環境(real)を統合したDocker環境です。

## 使用方法

### スクリプトを使用した起動(推奨)

リポジトリルートから以下のコマンドを実行します。

```bash
# シミュレーション環境(デフォルト: ER-Force) + ssl-log-recorder自動起動
./scripts/docker-dev.sh

# シミュレーション環境(grSim)
./scripts/docker-dev.sh --sim grsim

# シミュレーション環境(ER-Force、明示指定)
./scripts/docker-dev.sh --sim erforce

# 実機環境 + ssl-log-recorder自動起動
./scripts/docker-dev.sh real

# バックグラウンド起動
./scripts/docker-dev.sh -d
./scripts/docker-dev.sh --sim grsim -d

# robot-manager なしで起動
./scripts/docker-dev.sh --no-debug

# 停止
./scripts/docker-dev.sh down
```

**注1**: `up` 実行時（フォアグラウンド/バックグラウンドの両方）に
`ssl-log-recorder` (`robocupssl/ssl-log-recorder:latest`) が自動的に起動し、
ログはリポジトリルートに保存されます。

**注2**: `ssl-log-recorder` は `./scripts/docker-dev.sh down` 実行時に停止します。
`up` を Ctrl+C で終了した場合は recorder は継続起動します。

**注3**: `robot-manager` は Docker Compose サービスとして常時定義されています。

- URL: <http://localhost:8092>
- `--no-debug` オプションで無効化可能（`robot-manager` のみ停止）
- イメージは [ibis-ssl/Orion_CM4](https://github.com/ibis-ssl/Orion_CM4) リポジトリで管理・ビルドされます（`ghcr.io/ibis-ssl/robot-manager:latest`）

### robot-manager の手動操作

`robot-manager` を個別に操作したい場合は、以下のスクリプトを使用できます。

```bash
# robot-manager を起動
./scripts/start-debug-tools.sh

# robot-manager を停止
./scripts/stop-debug-tools.sh
```

### Docker Composeコマンドでの直接起動

```bash
# シミュレーション環境(ER-Force)
docker compose -f docker/dev/docker-compose.yaml --profile sim-erforce up

# シミュレーション環境(grSim)
docker compose -f docker/dev/docker-compose.yaml --profile sim-grsim up

# 実機環境
VISION_PORT=10006 docker compose -f docker/dev/docker-compose.yaml up

# 停止
docker compose -f docker/dev/docker-compose.yaml down
```

## sim/real の差分

| 項目 | sim (ER-Force) | sim (grSim) | real |
|------|----------------|-------------|------|
| Vision ポート | 10020 | 10020 | 10006 |
| ssl-status-board | 有効 | 有効 | 無効 |
| profile | `sim-erforce` | `sim-grsim` | なし |

環境変数 `VISION_PORT` とDocker Composeの `profiles` 機能を使用して切り替えています。

## サービス一覧

- **ssl-game-controller**: RoboCup SSLのゲームコントローラー
- **ssl-vision-client**: SSL Vision クライアント
- **ssl-status-board**: ステータスボード(simのみ)
- **autoref-tigers**: Tigers Mannheimのオートレフェリー
- **voicevox**: 音声合成エンジン
- **robot-manager**: ROS非依存のロボット管理Webアプリ（Start/Stop/Status）
- **erforce-sim**: ER-Forceシミュレータ（`sim-erforce` profile時のみ）
- **grsim**: grSimシミュレータ（`sim-grsim` profile時のみ）

## 設定ファイル

- `docker-compose.yaml`: サービス定義
- `.env`: 環境変数(Visionポートなどのデフォルト設定)
- `config/`: ゲームコントローラー設定ファイル
