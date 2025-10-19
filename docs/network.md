# ネットワーク設定

## ROS関連

<https://autowarefoundation.github.io/autoware-documentation/pr-347/installation/additional-settings-for-developers/#network-settings-for-ros-2>

### ローカルホストでマルチキャスト

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
    crane_sender -- UPD to 192.168.20.1xx --> ibisInterface

    ibisInterface -- UPD to 192.168.20.1xx --> SwitchingHub
    SwitchingHub -- UPD to 192.168.20.1xx --> Router
    Router -- AICommand --> Robots
    Robots -- RobotFeedback --> Router
    Router -- UPD to 192.168.20.1xx --> SwitchingHub
    SwitchingHub -- RobotFeedback UPD Multicast --> ibisInterface
    ibisInterface -- RobotFeedback UPD Multicast --> crane_robot_receiver
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
    - 11111
      - ibisがポート被り防止に使うことが多い
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
