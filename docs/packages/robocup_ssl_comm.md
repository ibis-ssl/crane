# robocup_ssl_comm

## 概要

RoboCup Small Size League（SSL）の**公式通信プロトコル処理**を担うパッケージです。SSL Vision・Referee プロトコルの受信、grSim通信、およびSSL標準プロトコルとROS 2システムの橋渡し機能を提供します。

## 主要機能

- **SSL Vision受信**: カメラシステムからのロボット・ボール位置データ受信
- **SSL Referee受信**: 審判システムからの試合状況・判定データ受信
- **grSim通信**: grSimシミュレータとの双方向通信
- **プロトコル変換**: SSL Protobuf ⇔ ROS 2メッセージ変換

## アーキテクチャ上の役割

Craneシステムの**外部通信インターフェース**として、SSL標準プロトコルを介した外部システムとの通信を担い、Craneシステム内部への統合されたデータ流入を実現します。

## コンポーネント

### GameControllerComponent

- **Referee通信**: SSL Referee プロトコルの受信・解析
- **試合状況管理**: ゲーム状態・判定・タイマー情報の処理
- **コマンド配信**: 試合制御コマンドのROS 2配信

### GrsimComponent  

- **grSim通信**: シミュレータとの双方向データ交換
- **ロボット制御**: シミュレーション環境でのロボット制御
- **環境操作**: ボール・ロボット配置の操作

## SSL プロトコル対応

### SSL Vision Protocol

```cpp
// SSL Vision データ受信
void onVisionData(const SSL_WrapperPacket& packet) {
  // Detection データ処理
  if (packet.has_detection()) {
    processDetectionFrame(packet.detection());
  }

  // Geometry データ処理  
  if (packet.has_geometry()) {
    processGeometryData(packet.geometry());
  }
}
```

### SSL Referee Protocol

```cpp
// Referee データ受信
void onRefereeData(const Referee& referee_msg) {
  // ゲーム状態の更新
  updateGameState(referee_msg.stage(), referee_msg.command());

  // チーム情報の処理
  processTeamInfo(referee_msg.blue(), referee_msg.yellow());

  // ゲームイベントの処理
  processGameEvents(referee_msg.game_events());
}
```

## 通信設定

### ネットワーク設定

```yaml
ssl_vision:
  multicast_address: "224.5.23.2"
  port: 10006

ssl_referee:  
  multicast_address: "224.5.23.1"
  port: 10003

grsim:
  host: "127.0.0.1"
  command_port: 20011
  blue_status_port: 30012
  yellow_status_port: 30013
```

### 起動方法

```bash
# SSL通信システム起動
ros2 launch robocup_ssl_comm comm.launch.py

# 個別コンポーネント起動
ros2 run robocup_ssl_comm game_controller_node
ros2 run robocup_ssl_comm grsim_node
```

## データフロー

### Vision → World Model

```text
SSL Camera → Vision Protocol → robocup_ssl_comm →
crane_world_model_publisher → WorldModel
```

### Referee → Session Control

```text
SSL Referee → Referee Protocol → robocup_ssl_comm →
crane_tactic_coordinator → Strategy Selection
```

### Commands → Robots

```text
Robot Commands → robocup_ssl_comm → grSim Protocol →
grSim/Real Robots
```

## 依存関係

### プロトコル依存

- **robocup_ssl_msgs**: SSL公式メッセージ定義
- **protobuf**: Protocol Buffers ライブラリ

### システム依存

- **crane_comm**: 通信ユーティリティ
- **rclcpp_components**: ROS 2コンポーネント機能

## プロトコル互換性

### SSL Vision 2023 Protocol

- **Detection Frame**: ロボット・ボール検出データ
- **Geometry Data**: フィールド幾何情報
- **Tracking Frame**: 追跡済みオブジェクト情報

### SSL Referee 2023 Protocol  

- **Game State**: HALT、STOP、RUNNING等の試合状態
- **Game Commands**: キックオフ、フリーキック等のコマンド
- **Team Info**: 得点、カード、タイムアウト情報

### grSim Protocol

- **Robot Commands**: 速度・キック指令
- **Ball Replacement**: ボール配置指令
- **Robot Replacement**: ロボット配置指令

## 最近の開発状況

### 2025年の主要変更

- **プロトコル更新**: SSL 2025仕様に合わせたReferee/Trackerオプション拡張
- **通信信頼性向上**: Tracker再接続の自動フォールバックを追加
- **パフォーマンス改善**: マルチスレッド送信キューの遅延削減
- **新機能対応**: JapanOpenで利用したマルチポート設定をデフォルト化

### 開発活発度

🟢 **安定**: SSL標準プロトコル対応として成熟しており、新しいSSL規格への対応やプロトコル信頼性の向上が継続的に行われています。

### SSL規格対応

- **2023年規格**: 完全対応済み
- **2024年規格**: 対応済み
- **2025年規格**: Tracker拡張含め対応済み

## トラブルシューティング

### よくある問題

1. **Vision信号なし**: ネットワーク設定・ファイアウォール確認
2. **Referee接続失敗**: マルチキャストアドレス・ポート確認
3. **grSim通信エラー**: grSim起動状況・ポート競合確認

### デバッグ方法

```bash
# ネットワーク通信確認
ros2 topic echo /vision
ros2 topic echo /referee

# grSim接続確認  
ros2 topic echo /robot_commands
```

---

**関連パッケージ**: [robocup_ssl_msgs](./robocup_ssl_msgs.md) | [crane_world_model_publisher](./crane_world_model_publisher.md) | [crane_sender](./crane_sender.md)
