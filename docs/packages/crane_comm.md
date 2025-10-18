# crane_comm

## 概要

Craneシステムの**通信・ネットワークユーティリティライブラリ**として、UDP通信、マルチキャスト、診断機能付きパブリッシャー、時刻管理などの通信基盤機能を提供するパッケージです。ROS 2の通信機能を拡張し、SSL競技で必要な高速・信頼性の高い通信を実現しています。

## 主要機能

- **UDP通信**: 高速なUDP送受信機能
- **マルチキャスト**: グループ通信サポート
- **診断機能付きパブリッシャー**: 通信状態監視機能付きROS 2パブリッシャー
- **パラメータイベント**: 動的パラメータ変更の検知・処理
- **時刻管理**: システム全体の時刻同期ユーティリティ
- **拡張ノードハンドル**: ROS 2ノード機能の拡張

## アーキテクチャ上の役割

Craneシステムの**通信基盤層**として、ROS 2標準の通信機能を拡張し、SSL競技特有の要求（低レイテンシ、診断機能、マルチキャスト等）に対応した通信機能を提供します。

## 主要コンポーネント

### UDP通信

```cpp
class UDPSender {
public:
  UDPSender(const std::string& host, int port);
  void send(const std::vector<uint8_t>& data);
  void sendString(const std::string& message);
};

class MulticastReceiver {
public:
  MulticastReceiver(const std::string& group, int port);
  std::vector<uint8_t> receive();
  void setNonBlocking(bool non_blocking);
};
```

### 診断機能付きパブリッシャー

```cpp
template<typename MessageType>
class DiagnosedPublisher {
public:
  DiagnosedPublisher(rclcpp::Node* node, const std::string& topic_name);

  void publish(const MessageType& message);
  void updateDiagnostics();

  // 診断状態の取得
  bool isHealthy() const;
  double getPublishRate() const;
  size_t getDroppedMessages() const;

private:
  rclcpp::Publisher<MessageType>::SharedPtr publisher_;
  diagnostic_updater::DiagnosticUpdater diagnostic_updater_;
};
```

### パラメータイベント機能

```cpp
class ParameterWithEvent {
public:
  template<typename T>
  ParameterWithEvent(rclcpp::Node* node, const std::string& name, const T& default_value);

  // 値の取得
  template<typename T>
  T getValue() const;

  // 変更通知コールバック
  void setCallback(std::function<void()> callback);

  // 動的更新
  template<typename T>
  void setValue(const T& value);
};
```

### 拡張ノードハンドル

```cpp
class NodeHandle {
public:
  NodeHandle(rclcpp::Node* node);

  // 診断機能付きパブリッシャー生成
  template<typename MessageType>
  auto createDiagnosedPublisher(const std::string& topic_name, size_t qos = 10);

  // パラメータイベント対応
  template<typename T>
  ParameterWithEvent declareParameterWithEvent(const std::string& name, const T& default_value);

  // ストリーム機能
  template<typename StreamType>
  auto createStream(const std::string& name);
};
```

### 時刻管理

```cpp
namespace time {
  // 高精度タイマー
  class HighResolutionTimer {
  public:
    void start();
    double elapsed() const;  // 秒単位
    void reset();
  };

  // 時刻同期
  double getSystemTime();
  double getRosTime(rclcpp::Node* node);
  double getTimeDifference(double t1, double t2);
}
```

## 依存関係

### パッケージ依存

- **rclcpp**: ROS 2 C++クライアントライブラリ
- **diagnostic_updater**: 診断機能

### システム依存

- **標準ライブラリ**: STL、ネットワークAPI
- **システムソケット**: UDP/TCPソケット

## 使用方法

### UDP通信

```cpp
#include "crane_comm/udp_sender.hpp"

UDPSender sender("192.168.1.100", 10001);
std::vector<uint8_t> data = {0x01, 0x02, 0x03};
sender.send(data);
```

### 診断機能付きパブリッシャー

```cpp
#include "crane_comm/diagnosed_publisher.hpp"

DiagnosedPublisher<geometry_msgs::msg::Twist> cmd_pub(this, "cmd_vel");

geometry_msgs::msg::Twist cmd;
cmd.linear.x = 1.0;
cmd_pub.publish(cmd);

// 診断状態確認
if (!cmd_pub.isHealthy()) {
    RCLCPP_WARN(get_logger(), "Publisher unhealthy: rate=%.1f Hz",
                cmd_pub.getPublishRate());
}
```

### パラメータイベント

```cpp
#include "crane_comm/parameter_with_event.hpp"

auto max_speed = declareParameterWithEvent("max_speed", 3.0);

max_speed.setCallback([this]() {
    RCLCPP_INFO(get_logger(), "Max speed changed to: %.1f",
                max_speed.getValue<double>());
    updateRobotLimits();
});
```

## パフォーマンス特性

- **UDP通信遅延**: <1ms（ローカルネットワーク）
- **診断更新頻度**: 1-10Hz（設定可能）
- **パラメータ更新遅延**: <10ms

## 最近の開発状況

🟡 **中活動**: crane_basicsからの分離後、診断機能の強化、マルチキャスト通信の最適化が進められています。特にネットワーク状態の監視機能が充実し、通信品質の可視化が改善されました。

---

**関連パッケージ**: [crane_sender](./crane_sender.md) | [robocup_ssl_comm](./robocup_ssl_comm.md) | [crane_msg_wrappers](./crane_msg_wrappers.md)
