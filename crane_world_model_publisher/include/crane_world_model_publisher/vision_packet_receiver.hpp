// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__VISION_PACKET_RECEIVER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__VISION_PACKET_RECEIVER_HPP_

#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <crane_comm/multicast.hpp>
#include <functional>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
struct VisionPacketMetadata
{
  double t_capture;    // Camera capture timestamp
  double t_sent;       // Packet sent timestamp
  rclcpp::Time t_received;  // ROS2 receive timestamp
  uint32_t frame_number;
  std::string camera_id;
  size_t packet_size;
};

struct VisionPacket
{
  SSL_WrapperPacket packet;
  VisionPacketMetadata metadata;
};

enum class VisionReceiverStatus {
  INACTIVE,      // Not receiving packets
  ACTIVE,        // Actively receiving packets
  DEGRADED,      // Receiving but with issues (low frequency, errors)
  ERROR          // Connection error or critical failure
};

class VisionPacketReceiver
{
public:
  using PacketCallback = std::function<void(const VisionPacket &)>;
  using StatusCallback = std::function<void(VisionReceiverStatus, const std::string &)>;

  explicit VisionPacketReceiver(rclcpp::Node & node);
  ~VisionPacketReceiver() = default;

  // Connection management
  auto start(const std::string & address = "224.5.23.2", int port = 10020) -> bool;
  auto stop() -> void;
  auto restart() -> bool;

  // Callback registration
  auto setPacketCallback(PacketCallback callback) -> void { packet_callback_ = callback; }
  auto setStatusCallback(StatusCallback callback) -> void { status_callback_ = callback; }

  // Status monitoring
  [[nodiscard]] auto getStatus() const -> VisionReceiverStatus { return status_; }
  [[nodiscard]] auto getStatusMessage() const -> const std::string & { return status_message_; }
  [[nodiscard]] auto isActive() const -> bool { return status_ == VisionReceiverStatus::ACTIVE; }
  [[nodiscard]] auto hasRecentPacket(double max_age_seconds = 1.0) const -> bool;

  // Packet processing
  auto processIncomingPackets() -> void;

  // Statistics
  [[nodiscard]] auto getPacketCount() const -> uint64_t { return packet_count_; }
  [[nodiscard]] auto getPacketRate() const -> double { return packet_rate_; }
  [[nodiscard]] auto getAverageLatency() const -> double { return average_latency_ms_; }
  [[nodiscard]] auto getDroppedPacketCount() const -> uint64_t { return dropped_packet_count_; }

  // Network configuration
  [[nodiscard]] auto getAddress() const -> const std::string & { return address_; }
  [[nodiscard]] auto getPort() const -> int { return port_; }

  // Quality metrics
  [[nodiscard]] auto getConnectionQuality() const -> double;  // [0.0, 1.0]
  auto resetStatistics() -> void;

  // Advanced features
  auto setPacketBufferSize(size_t buffer_size) -> void { max_buffer_size_ = buffer_size; }
  auto enablePacketValidation(bool enable) -> void { validate_packets_ = enable; }
  auto setExpectedPacketRate(double rate_hz) -> void { expected_packet_rate_ = rate_hz; }

private:
  rclcpp::Node & node_;

  // Network components
  std::unique_ptr<multicast::MulticastReceiver> receiver_;
  std::string address_;
  int port_;

  // Status tracking
  VisionReceiverStatus status_;
  std::string status_message_;

  // Callbacks
  PacketCallback packet_callback_;
  StatusCallback status_callback_;

  // Packet processing
  std::queue<VisionPacket> packet_buffer_;
  size_t max_buffer_size_;
  bool validate_packets_;

  // Statistics
  uint64_t packet_count_;
  uint64_t dropped_packet_count_;
  double packet_rate_;
  double average_latency_ms_;
  double expected_packet_rate_;

  // Timing
  rclcpp::Time last_packet_time_;
  rclcpp::Time statistics_start_time_;
  rclcpp::Time last_statistics_update_;

  // Internal methods
  auto updateStatus(VisionReceiverStatus new_status, const std::string & message) -> void;
  auto processRawPacket(const std::vector<uint8_t> & raw_data) -> bool;
  auto validatePacket(const SSL_WrapperPacket & packet) const -> bool;
  auto calculatePacketMetadata(const SSL_WrapperPacket & packet) const -> VisionPacketMetadata;
  auto updateStatistics() -> void;
  auto checkPacketRate() -> void;
  auto notifyStatusChange() -> void;

  // Buffer management
  auto addPacketToBuffer(const VisionPacket & packet) -> void;
  auto clearOldPackets() -> void;

  // Network diagnostics
  auto diagnoseNetworkIssues() -> std::string;
  auto checkNetworkInterface() const -> bool;
};

// Utility class for monitoring multiple vision receivers
class VisionReceiverMonitor
{
public:
  explicit VisionReceiverMonitor(rclcpp::Node & node);
  ~VisionReceiverMonitor() = default;

  // Receiver management
  auto addReceiver(const std::string & name, std::shared_ptr<VisionPacketReceiver> receiver)
    -> void;
  auto removeReceiver(const std::string & name) -> void;
  auto getReceiver(const std::string & name) const -> std::shared_ptr<VisionPacketReceiver>;

  // Monitoring
  auto startMonitoring(double check_interval_seconds = 1.0) -> void;
  auto stopMonitoring() -> void;
  [[nodiscard]] auto isMonitoring() const -> bool { return monitoring_active_; }

  // Health reporting
  [[nodiscard]] auto getOverallHealth() const -> double;  // [0.0, 1.0]
  [[nodiscard]] auto getHealthReport() const -> std::string;
  [[nodiscard]] auto getActiveReceiverCount() const -> size_t;

  // Callbacks
  using HealthCallback = std::function<void(const std::string &, double)>;
  auto setHealthCallback(HealthCallback callback) -> void { health_callback_ = callback; }

private:
  rclcpp::Node & node_;

  std::map<std::string, std::shared_ptr<VisionPacketReceiver>> receivers_;
  bool monitoring_active_;
  rclcpp::TimerBase::SharedPtr monitoring_timer_;
  HealthCallback health_callback_;

  auto performHealthCheck() -> void;
  auto generateHealthReport() -> void;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISION_PACKET_RECEIVER_HPP_