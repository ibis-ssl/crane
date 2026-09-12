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

ホストのループバック (`lo`) にマルチキャストが設定されていない場合、**マルチキャストパケットが物理 Wi-Fi / LAN へ漏洩し、Wi-Fi 帯域を極端に圧迫（Wi-Fi アクセスポイントがダウンすることもある）** します。

これを防ぐため `scripts/setup-multicast.sh` は次の2段構えで隔離します（`sudo ./scripts/setup-multicast.sh` で手動実行可能）：

1. `224.0.0.0/4` の送信経路を `lo` に向ける（ルーティングベース）
2. `224.5.23.0/24` の `lo` 以外への送出を **iptables で強制遮断**（パケットフィルタベース）

**1. だけでは不十分**な点に注意してください。`ssl-game-controller` など一部の公式ツールは、送信元アドレスを
ホストの各ネットワークインターフェースのIPに明示バインドしてマルチキャストを送信するため、ルーティングテーブルの
設定を無視して物理 Wi-Fi/LAN に直接パケットを送出します（実際にこれが原因で Wi-Fi アクセスポイントが
高頻度マルチキャストにより過負荷でダウンした実績があります）。そのため 2. の iptables による遮断を
独立した保護層として併用しています。コンテナ間の通信は同一ホスト内の `IP_MULTICAST_LOOP` により
物理インターフェースの状態に関わらず継続されるため、`lo` 以外への送出を遮断するだけで機能に影響はありません。

※ `./scripts/docker-dev.sh` の sim モード (`up`) 実行時、未設定の項目があれば自動で検知して
設定スクリプトを実行し、パスワード入力を求めます。`scripts/scenario_test/run_test.sh` /
`make scenario-test-docker-up` / `scripts/match-vs-tigers/run_local.sh` など、同様に
`network_mode: host` でシミュレータを起動する他のスクリプトも
`scripts/ensure-sim-network-confined.sh` 経由で同じ保護を適用します。

※ この隔離設定は **sim モード専用** です。`./scripts/docker-dev.sh real` を実行すると、
残っていないことを確認・警告した上で起動します（real モードで隔離が残っていると、実機の
Vision/Referee を受信できずサイレントに機能しなくなるため）。

### ネットワークやマシンに負荷をかけずにテストする方法

用途に合わせて以下の方法を使い分けることで、ネットワークやシステムへの負荷を最小限に抑えられます。

1. **最小構成でのシミュレータ起動 (`--minimal`)**
   重い音声エンジン (`voicevox`) やレコーダー、実機管理ツールを起動せず、開発に必要な5つのサービス（シミュレータ本体、Game Controller: `8081`、Vision Client: `8082`、Web Debugger: `8090`、AutoRef）のみを起動します。

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
