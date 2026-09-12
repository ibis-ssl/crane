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

## ネットワーク負荷対策と注意点

### マルチキャスト設定（必須）

シミュレータ環境の Docker コンテナは `network_mode: host` で動作し、マルチキャスト（SSL Vision: `224.5.23.2`, Game Controller: `224.5.23.1`）を使用します。

ホストのループバック (`lo`) にマルチキャストが設定されていない場合、**マルチキャストパケットが物理 Wi-Fi / LAN へ漏洩し、Wi-Fi 帯域を極端に圧迫（Wi-Fi が遅延・切断）** します。

これを防ぐため、`lo` へのマルチキャストルート設定が必要です：

```bash
sudo ./scripts/setup-multicast.sh
```

※ `./scripts/docker-dev.sh up` 実行時、未設定の場合は自動で検知してブロックし、設定スクリプトを実行してパスワード入力を求めます。

### ネットワークやマシンに負荷をかけずにテストする方法

用途に合わせて以下の方法を使い分けることで、ネットワークやシステムへの負荷を最小限に抑えられます。

1. **最小構成でのシミュレータ起動 (`--minimal`)**
   デバッガやレコーダー、音声エンジン等を起動せず、シミュレータ本体、Game Controller、Vision Client (Webビューア: <http://localhost:8082>) のみ起動します。

   ```bash
   ./scripts/docker-dev.sh --minimal
   ```

2. **シナリオテスト (推奨・自動テスト)**
   ヘッドレスかつ短時間のコンテナ起動で、ロボットの動作や反則判定を自動検証します。

   ```bash
   make scenario-test TEST=STOP_ROBOT_SPEED
   ```

3. **単体テスト (`colcon test`)**
   Docker を立ち上げず、ローカルの C++ 単体テストを実行します（ネットワーク負荷ゼロ）。

   ```bash
   colcon test --packages-select crane_physics crane_robot_skills
   ```

## 閲覧

- [game-controller](http://localhost:8081)
- [vision client](http://localhost:8082)
- [status board](http://localhost:8083)
