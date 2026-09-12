# ネットワーク設定

## ROS関連

<https://autowarefoundation.github.io/autoware-documentation/pr-347/installation/additional-settings-for-developers/#network-settings-for-ros-2>

### ローカルホストでマルチキャスト

Docker 開発環境（`network_mode: host`）では、ホスト内で完結する UDP multicast
（SSL Vision/Referee/Tracker、ROS 2 DDS など）を正しく疎通させ、かつ物理 Wi-Fi/LAN への
漏洩を防ぐために、以下2つの設定が必要:

1. lo インターフェイスのマルチキャスト有効化と `224.0.0.0/4` のループバック向けルート
2. `224.5.23.0/24`（SSL Vision/Referee/Tracker）の lo 以外への送出を iptables で遮断

一括で適用するヘルパーを用意している:

```bash
sudo ./scripts/setup-multicast.sh
```

このスクリプトは冪等で、以下を実行する（sudo パスワードの入力が必要）:

- `sudo ip link set multicast on lo`
- `sudo ip route replace 224.0.0.0/4 dev lo`
- `sudo iptables -I OUTPUT -d 224.5.23.0/24 ! -o lo -j DROP`（コメント `crane-sim-multicast-confine` 付き）

**1. のルート設定だけでは不十分**なので注意すること。`ssl-game-controller` の referee 送信部
（`internal/app/publish/publisher.go`）は `net.InterfaceAddrs()` でホストの全インターフェースの
IPを列挙し、各インターフェースのIPに送信元アドレスを明示バインドしてマルチキャストを送信する。
この方式はルーティングテーブルの参照より優先されるため、`224.0.0.0/4 dev lo` を設定していても
Wi-Fi インターフェースに直接パケットが送出される。実際にこれが原因で、シミュレータ起動中に
Wi-Fi アクセスポイントが高頻度マルチキャストにより過負荷でダウンした事例がある
（`processTick: Hook messageGen unresponsive! Failed to sent` のログ大量出力が同時に発生する
= referee メッセージ送信のホットループが詰まっている兆候）。そのため 2. の iptables による
明示的な遮断を、ルーティングとは独立した保護層として併用している。

**OS 再起動や `iptables` のフラッシュで失われる**ので、再起動後に
`erforce-sim | Sending UDP datagram failed` や `ssl-game-controller | messageGen unresponsive`
が大量に出たら、まず Wi-Fi の状態を確認しつつ再実行すること。

**この隔離設定が正しく効いている場合の正常なログ（故障ではない）**: `ssl-game-controller` は
起動時に検出した全インターフェースぶんの接続を張ろうとするため、コンテナログに
`Could not write referee message on 192.168.10.x:...: operation not permitted` のような行が
起動直後に**数行だけ**出ることがある（Wi-FiインターフェースのIPへ束縛した接続が iptables に
遮断されてすぐに切断される）。これは遮断が機能している証拠であり、繰り返し出続けるものではない。
`erforce-sim`/`grSim` は単一ソケットでOSのルーティングテーブルに従って送信するだけなので
（インターフェースを明示バインドしない）、この iptables ルールの影響を受けず通常どおり動作する。
**上記の「大量に出る」系の症状（`Sending UDP datagram failed` がVisionレート＝毎秒数十回続く、
`messageGen unresponsive` が延々と出続ける）とは別物であり、後者が出た場合は
`224.0.0.0/4 dev lo` ルートが外れている（1. が失われている）可能性が高い。
そのときの対処は `sudo ./scripts/setup-multicast.sh` の再実行であり、
iptables 遮断ルール自体を外すことではない（外すと本来のWi-Fi漏洩問題が復活する）。**

この隔離設定は **sim 専用**。`scripts/docker-dev.sh real` 実行時は
`scripts/restore-real-network.sh` により、iptables 遮断ルールを自動解除し、
`224.0.0.0/4 dev lo` ルートが残っていれば警告して起動を中断する
（残ったままだと実機の Vision/Referee をサイレントに受信できなくなるため）。

`scripts/docker-dev.sh` 以外にも、`network_mode: host` でシミュレータ系コンテナを起動する
`scripts/scenario_test/run_test.sh`・`make scenario-test-docker-up`・
`scripts/match-vs-tigers/run_local.sh` は `scripts/ensure-sim-network-confined.sh` 経由で
同じ隔離設定を起動前に適用する。

手動で個別に実行したい場合は:

```bash
sudo ip link set multicast on lo
```

### マルチキャストアドレスとデバイスの対応の確認

```bash
netstat -g
```

### マルチキャストアドレスへのルートの追加

```bash
sudo ip route add <address> dev <device>
```

```mermaid
graph TD
    subgraph official
        GameController[Game Controller]
        AutoRef[Auto Ref]
        Vision[SSL Vision]
    end

    OfficialHub[大会用スイッチングハブ]

    subgraph AIPC
        OfficialInterface[大会サーバー用Interface]
        ibisInterface[ロボット用Interface]
        crane[crane (Core AI Logic)]
        crane_sender[crane_sender]
        crane_robot_receiver[crane_robot_receiver]
    end

    SwitchingHub[スイッチングハブ]
    Router[ルーター]

    Robots[ロボット]
    PC[開発PC]

    Net[インターネット]

    GameController -- UDP Multicast --> OfficialHub
    AutoRef -- UDP Multicast --> OfficialHub
    Vision -- UDP Multicast --> OfficialHub

    OfficialHub -- UDP Multicast --> SwitchingHub
    SwitchingHub -- UDP Multicast --> OfficialInterface
    OfficialInterface -- UDP Multicast --> crane
    crane -- ROS --> crane_sender
    crane_sender -- UDP to 192.168.20.1xx --> ibisInterface

    ibisInterface -- UDP to 192.168.20.1xx --> SwitchingHub
    SwitchingHub -- UDP to 192.168.20.1xx --> Router
    Router -- AICommand --> Robots
    Robots -- RobotFeedback --> Router
    Router -- UDP to 192.168.20.1xx --> SwitchingHub
    SwitchingHub -- RobotFeedback UDP Multicast --> ibisInterface
    ibisInterface -- RobotFeedback UDP Multicast --> crane_robot_receiver
    crane_robot_receiver -- ROS  --> crane

```

## アドレス・ポートなど

### 公式ツールなど

- Vision
  - アドレス：224.5.23.2
  - ポート
    - 10006
      - 本番で使われることが多い
      - `ssl-vision`のデフォルトポート
    - 10020
      - シミュレーションなどで使われることが多い
      - `grSim`のデフォルトポート
- Game Controller
  - アドレス：224.5.23.1
  - ポート
    - 10003
      - デフォルト
      - 本番で使われる
    - 11003
      - grSimなどのシミュレーション環境でのデフォルト（crane設定）
    - 11111
      - ポート被り防止などに使用可能
- Tracker
  - アドレス：224.5.23.2
  - ポート：10010
  - 参考：<https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/proto/ssl_vision_detection_tracked.proto#L8>

### ibis

- ロボットのCM4
  - アドレス：192.168.20.100+機体番号
  - コマンド用ポート：12345
- ロボットからのフィードバック
  - アドレス：224.5.20.100
  - ポート：50100+機体番号
