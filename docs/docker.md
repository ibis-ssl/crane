# docker立ち上げ

## 準備

Docker Compose V2 は通常 Docker Engine に同梱されています (Docker Desktop や Linux 向けの Docker Engine インストールに含まれる `docker-compose-plugin` パッケージなど)。
最新のDocker環境では、`docker compose` (ハイフンなし) コマンドとして利用可能です。

もし個別にインストールまたはアップデートが必要な場合は、[公式Dockerドキュメント](https://docs.docker.com/compose/install/)を参照してください。
以前記載されていた手動ダウンロード・インストール方法は特定バージョン (`v2.5.0`) に固定されており、古くなっている可能性があります。

**注意:** Docker Compose V1 (ハイフンありの `docker-compose`) はサポートが終了しており、使用は推奨されません。

## 起動

### ツール群

各環境の起動手順は `docker/README.md` を参照してください。

### grSim

起動方法は `docs/setup.md` のGrSimセクションを参照してください。

## 閲覧

- [game-controller](http://localhost:8081)
- [vision client](http://localhost:8082)
- [status board](http://localhost:8083)
