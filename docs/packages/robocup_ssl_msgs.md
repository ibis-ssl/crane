# robocup_ssl_msgs

## 概要

RoboCup Small Size League（SSL）の**公式プロトコルメッセージ定義**を提供するパッケージです。SSL Vision・Referee・grSim・Simulation等の標準プロトコルをROS 2メッセージとして実装し、SSL準拠の通信インターフェースを実現します。

## 主要機能

- **SSL公式プロトコル**: Vision・Referee・grSim等の標準メッセージ実装
- **Protobuf統合**: Protocol Buffers定義からROS 2メッセージ自動生成
- **バージョン管理**: SSL規格変更への対応とバックワード互換性
- **型安全通信**: コンパイル時型チェックによる安全な通信

## アーキテクチャ上の役割

Craneシステムの**SSL標準メッセージ基盤**として、SSL公式プロトコルとROS 2システムの橋渡しを行い、標準準拠の通信を保証します。

## プロトコル定義

### SSL Vision Protocol

- **ssl_vision_wrapper.proto**: Vision データのラッパー
- **ssl_vision_detection.proto**: ロボット・ボール検出データ
- **ssl_vision_geometry.proto**: フィールド幾何情報
- **ssl_vision_detection_tracked.proto**: 追跡済みオブジェクト

### SSL Referee Protocol

- **ssl_gc_referee_message.proto**: 審判メッセージ
- **ssl_gc_common.proto**: 共通データ型
- **ssl_gc_game_event.proto**: ゲームイベント定義
- **ssl_gc_geometry.proto**: 幾何情報

### grSim Protocol

- **grSim_Packet.proto**: grSim通信パケット
- **grSim_Commands.proto**: ロボット制御コマンド
- **grSim_Replacement.proto**: オブジェクト配置
- **grSim_Robotstatus.proto**: ロボット状態情報

### SSL Simulation Protocol

- **ssl_simulation_control.proto**: シミュレーション制御
- **ssl_simulation_config.proto**: シミュレーション設定
- **ssl_simulation_robot_control.proto**: ロボット制御
- **ssl_simulation_robot_feedback.proto**: ロボットフィードバック

## メッセージ構造例

### Vision Detection

```protobuf
message SSL_DetectionFrame {
  required uint32 frame_number = 1;
  required double t_capture = 2;
  required double t_sent = 3;
  required uint32 camera_id = 4;
  repeated SSL_DetectionBall balls = 5;
  repeated SSL_DetectionRobot robots_yellow = 6;
  repeated SSL_DetectionRobot robots_blue = 7;
}
```

### Referee Message

```protobuf
message Referee {
  required uint64 packet_timestamp = 1;
  required Stage stage = 2;
  optional sint32 stage_time_left = 3;
  required Command command = 4;
  required uint32 command_counter = 5;
  required uint64 command_timestamp = 6;
  required TeamInfo yellow = 7;
  required TeamInfo blue = 8;
}
```

### Robot Command (grSim)

```protobuf
message grSim_Robot_Command {
  required uint32 id = 1;
  required float kickspeedx = 2;
  required float kickspeedy = 3;
  required float veltangent = 4;
  required float velnormal = 5;
  required float velangular = 6;
  required bool spinner = 7;
  required bool wheelsspeed = 8;
}
```

## ROS 2統合

### 自動コード生成

```cmake
# CMakeLists.txt
rosidl_generate_interfaces(${PROJECT_NAME}
  "proto/ssl_vision_wrapper.proto"
  "proto/ssl_gc_referee_message.proto"
  "proto/grSim_Packet.proto"
  # ... 他のprotoファイル
  DEPENDENCIES builtin_interfaces
)
```

### 使用例（C++）

```cpp
#include "robocup_ssl_msgs/msg/ssl_wrapper_packet.hpp"
#include "robocup_ssl_msgs/msg/referee.hpp"

// Vision データ受信
auto vision_subscription = create_subscription<robocup_ssl_msgs::msg::SSL_WrapperPacket>(
  "vision", 10, [this](const robocup_ssl_msgs::msg::SSL_WrapperPacket::SharedPtr msg) {
    processVisionData(*msg);
  });

// Referee データ受信  
auto referee_subscription = create_subscription<robocup_ssl_msgs::msg::Referee>(
  "referee", 10, [this](const robocup_ssl_msgs::msg::Referee::SharedPtr msg) {
    processRefereeData(*msg);
  });
```

## SSL規格対応

### 対応バージョン

- **SSL Vision 2023**: 完全対応
- **SSL Referee 2023**: 完全対応
- **grSim 2023**: 完全対応
- **SSL Simulation 2023**: 対応

### 新規格対応手順

1. 新しい.protoファイルの追加
2. CMakeLists.txtの更新
3. 依存関係の調整
4. 互換性テストの実行

## 依存関係

### ビルド依存

- **protobuf-dev**: Protocol Buffers開発環境
- **rosidl_default_generators**: ROS 2メッセージ生成

### 実行依存

- **rosidl_default_runtime**: ROS 2メッセージランタイム

## パフォーマンス特性

### メッセージサイズ

- **Vision Frame**: ~1-5KB（標準）
- **Referee Message**: ~1KB
- **Robot Commands**: ~100B（ロボット1台）

### 通信頻度

- **Vision**: 60Hz
- **Referee**: 10Hz
- **Commands**: 60Hz

## 最近の開発状況

### 2024年の主要変更

- **SSL 2024対応**: 新しいプロトコル仕様への対応
- **パフォーマンス最適化**: メッセージ処理の高速化
- **互換性向上**: 旧バージョンとの互換性維持
- **新機能追加**: SSL新機能のメッセージ定義

### 開発活発度

🟢 **安定**: SSL標準メッセージとして成熟しており、SSL規格変更への対応とメッセージ処理性能の向上が継続的に行われています。

### 標準準拠

- **SSL 2023**: 100%準拠
- **SSL 2024**: 100%準拠
- **下位互換性**: 2022年以降のバージョンと互換

## 使用上の注意

### メッセージサイズ制限

- UDP通信での最大サイズ制約
- 大きなフレームでの分割処理

### エンディアン対応

- 異なるアーキテクチャ間での互換性
- Protobufの自動処理

---

**関連パッケージ**: [robocup_ssl_comm](./robocup_ssl_comm.md) | [crane_msgs](./crane_msgs.md)
