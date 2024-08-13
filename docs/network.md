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

## インターネット接続とロボット接続の共存

ロボットのアドレスに対して静的ルーティングを設定する

`/etc/netplan/01-network-manager-all.yaml`

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    enp3s0:
      addresses:
        - 192.168.1.2/24
      gateway4: 192.168.1.1
      routes:
        - to: 192.168.2.0/24
          via: 192.168.1.1
        - to: 192.168.3.0/24
          via: 192.168.1.1
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
        crane[crane]
        sender[real_sender]
        receiver[robot_receiver]
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
    crane -- ROS --> sender
    sender -- UPD to 192.168.20.1xx --> ibisInterface

    ibisInterface -- UPD to 192.168.20.1xx --> SwitchingHub
    SwitchingHub -- UPD to 192.168.20.1xx --> Router
    Router -- AICommand --> Robots
    Robots -- RobotFeedback --> Router
    Router -- UPD to 192.168.20.1xx --> SwitchingHub
    SwitchingHub -- RobotFeedback UPD Multicast --> ibisInterface
    ibisInterface -- RobotFeedback UPD Multicast --> receiver
    receiver -- ROS  --> crane




```
