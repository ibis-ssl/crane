# Docker開発環境（統合版）

このディレクトリは、シミュレーション環境（sim）と実機環境（real）を統合したDocker環境です。

## 使用方法

### スクリプトを使用した起動（推奨）

リポジトリルートから以下のコマンドを実行します。

```bash
# シミュレーション環境（デフォルト）
./scripts/docker-dev.sh

# 実機環境
./scripts/docker-dev.sh real

# バックグラウンド起動
./scripts/docker-dev.sh -d
./scripts/docker-dev.sh real -d

# 停止
./scripts/docker-dev.sh down
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
- **ssl-status-board**: ステータスボード（simのみ）
- **autoref-tigers**: Tigers Mannheimのオートレフェリー
- **voicevox**: 音声合成エンジン
- **aivis-speech**: AIVIS音声エンジン

## 設定ファイル

- `docker-compose.yaml`: サービス定義
- `.env`: 環境変数（Visionポートなどのデフォルト設定）
- `config/`: ゲームコントローラー設定ファイル
