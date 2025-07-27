// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/vision_packet_receiver.hpp"

#include <algorithm>
#include <chrono>

namespace crane
{
VisionPacketReceiver::VisionPacketReceiver(rclcpp::Node & node)
: node_(node),
  address_("224.5.23.2"),
  port_(10020),
  status_(VisionReceiverStatus::INACTIVE),
  status_message_("Not started"),
  max_buffer_size_(100),
  validate_packets_(true),
  packet_count_(0),
  dropped_packet_count_(0),
  packet_rate_(0.0),
  average_latency_ms_(0.0),
  expected_packet_rate_(60.0),
  last_packet_time_(rclcpp::Clock(RCL_ROS_TIME).now()),
  statistics_start_time_(rclcpp::Clock(RCL_ROS_TIME).now()),
  last_statistics_update_(rclcpp::Clock(RCL_ROS_TIME).now())
{
}

auto VisionPacketReceiver::start(const std::string & address, int port) -> bool
{
  address_ = address;
  port_ = port;

  try {
    receiver_ = std::make_unique<multicast::MulticastReceiver>(address_, port_);
    updateStatus(VisionReceiverStatus::ACTIVE, "Successfully started vision packet receiver");
    resetStatistics();
    return true;
  } catch (const std::exception & e) {
    updateStatus(VisionReceiverStatus::ERROR, "Failed to start receiver: " + std::string(e.what()));
    return false;
  }
}

auto VisionPacketReceiver::stop() -> void
{
  receiver_.reset();
  updateStatus(VisionReceiverStatus::INACTIVE, "Receiver stopped");
}

auto VisionPacketReceiver::restart() -> bool
{
  stop();
  return start(address_, port_);
}

auto VisionPacketReceiver::hasRecentPacket(double max_age_seconds) const -> bool
{
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();
  auto age = (now - last_packet_time_).seconds();
  return age <= max_age_seconds;
}

auto VisionPacketReceiver::processIncomingPackets() -> void
{
  if (!receiver_ || status_ == VisionReceiverStatus::INACTIVE) {
    return;
  }

  // Check for network issues periodically
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();
  if ((now - last_statistics_update_).seconds() > 1.0) {
    updateStatistics();
    checkPacketRate();
    last_statistics_update_ = now;
  }

  // Process available packets
  try {
    const size_t max_packets_per_cycle = 10;  // Prevent blocking
    size_t processed_count = 0;

    while (processed_count < max_packets_per_cycle) {
      if (receiver_->available() == 0) {
        break;  // No more packets available
      }

      std::vector<char> raw_data(4096);  // SSL packets are typically < 4KB
      try {
        size_t bytes_received = receiver_->receive(raw_data);
        if (bytes_received > 0) {
          raw_data.resize(bytes_received);
          std::vector<uint8_t> packet_data(raw_data.begin(), raw_data.end());
          if (processRawPacket(packet_data)) {
            processed_count++;
          }
        }
      } catch (const std::exception & e) {
        // No data available or error
        break;
      }
    }
  } catch (const std::exception & e) {
    updateStatus(VisionReceiverStatus::ERROR, "Packet processing error: " + std::string(e.what()));
  }

  // Clear old packets from buffer
  clearOldPackets();
}

auto VisionPacketReceiver::getConnectionQuality() const -> double
{
  if (status_ == VisionReceiverStatus::INACTIVE || status_ == VisionReceiverStatus::ERROR) {
    return 0.0;
  }

  double quality = 1.0;

  // Factor in packet rate (expect ~60 Hz)
  if (packet_rate_ < expected_packet_rate_ * 0.5) {
    quality *= 0.3;  // Severely degraded
  } else if (packet_rate_ < expected_packet_rate_ * 0.8) {
    quality *= 0.7;  // Moderately degraded
  }

  // Factor in packet age
  if (!hasRecentPacket(2.0)) {
    quality *= 0.1;  // Very stale data
  } else if (!hasRecentPacket(0.5)) {
    quality *= 0.5;  // Somewhat stale data
  }

  // Factor in dropped packets
  if (packet_count_ > 0) {
    double drop_rate = static_cast<double>(dropped_packet_count_) / packet_count_;
    quality *= (1.0 - std::min(drop_rate, 0.5));  // Cap penalty at 50%
  }

  return std::max(0.0, std::min(1.0, quality));
}

auto VisionPacketReceiver::resetStatistics() -> void
{
  packet_count_ = 0;
  dropped_packet_count_ = 0;
  packet_rate_ = 0.0;
  average_latency_ms_ = 0.0;
  statistics_start_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
  last_statistics_update_ = statistics_start_time_;
}

auto VisionPacketReceiver::updateStatus(
  VisionReceiverStatus new_status, const std::string & message) -> void
{
  if (status_ != new_status || status_message_ != message) {
    status_ = new_status;
    status_message_ = message;
    notifyStatusChange();
  }
}

auto VisionPacketReceiver::processRawPacket(const std::vector<uint8_t> & raw_data) -> bool
{
  try {
    SSL_WrapperPacket packet;
    if (!packet.ParseFromArray(raw_data.data(), static_cast<int>(raw_data.size()))) {
      dropped_packet_count_++;
      return false;
    }

    if (validate_packets_ && !validatePacket(packet)) {
      dropped_packet_count_++;
      return false;
    }

    VisionPacket vision_packet;
    vision_packet.packet = packet;
    vision_packet.metadata = calculatePacketMetadata(packet);

    addPacketToBuffer(vision_packet);

    if (packet_callback_) {
      packet_callback_(vision_packet);
    }

    packet_count_++;
    last_packet_time_ = rclcpp::Clock(RCL_ROS_TIME).now();

    return true;
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(node_.get_logger(), "Failed to process packet: %s", e.what());
    dropped_packet_count_++;
    return false;
  }
}

auto VisionPacketReceiver::validatePacket(const SSL_WrapperPacket & packet) const -> bool
{
  // Basic validation: check if packet has either detection or geometry data
  if (!packet.has_detection() && !packet.has_geometry()) {
    return false;
  }

  // Validate detection frame if present
  if (packet.has_detection()) {
    const auto & detection = packet.detection();
    if (detection.camera_id() > 32) {
      return false;
    }
    if (detection.t_capture() <= 0.0 || detection.t_sent() <= 0.0) {
      return false;
    }
  }

  return true;
}

auto VisionPacketReceiver::calculatePacketMetadata(const SSL_WrapperPacket & packet) const
  -> VisionPacketMetadata
{
  VisionPacketMetadata metadata;
  metadata.t_received = rclcpp::Clock(RCL_ROS_TIME).now();

  if (packet.has_detection()) {
    metadata.t_capture = packet.detection().t_capture();
    metadata.t_sent = packet.detection().t_sent();
    metadata.frame_number = packet.detection().frame_number();
    metadata.camera_id = std::to_string(packet.detection().camera_id());
  } else {
    metadata.t_capture = 0.0;
    metadata.t_sent = 0.0;
    metadata.frame_number = 0;
    metadata.camera_id = "geometry";
  }

  metadata.packet_size = packet.ByteSizeLong();

  return metadata;
}

auto VisionPacketReceiver::updateStatistics() -> void
{
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();
  double elapsed = (now - statistics_start_time_).seconds();

  if (elapsed > 0.0) {
    packet_rate_ = static_cast<double>(packet_count_) / elapsed;
  }

  // Update average latency (simplified calculation)
  if (packet_count_ > 0) {
    // This would typically track actual latencies, simplified for now
    average_latency_ms_ = 15.0;  // Typical SSL-Vision latency
  }
}

auto VisionPacketReceiver::checkPacketRate() -> void
{
  if (packet_rate_ < expected_packet_rate_ * 0.3) {
    if (status_ == VisionReceiverStatus::ACTIVE) {
      updateStatus(VisionReceiverStatus::DEGRADED, "Low packet rate detected");
    }
  } else if (packet_rate_ > expected_packet_rate_ * 0.8) {
    if (status_ == VisionReceiverStatus::DEGRADED) {
      updateStatus(VisionReceiverStatus::ACTIVE, "Packet rate recovered");
    }
  }
}

auto VisionPacketReceiver::notifyStatusChange() -> void
{
  if (status_callback_) {
    status_callback_(status_, status_message_);
  }

  // Log status changes
  switch (status_) {
    case VisionReceiverStatus::ACTIVE:
      RCLCPP_INFO(node_.get_logger(), "Vision receiver: %s", status_message_.c_str());
      break;
    case VisionReceiverStatus::DEGRADED:
      RCLCPP_WARN(node_.get_logger(), "Vision receiver: %s", status_message_.c_str());
      break;
    case VisionReceiverStatus::ERROR:
      RCLCPP_ERROR(node_.get_logger(), "Vision receiver: %s", status_message_.c_str());
      break;
    case VisionReceiverStatus::INACTIVE:
      RCLCPP_DEBUG(node_.get_logger(), "Vision receiver: %s", status_message_.c_str());
      break;
  }
}

auto VisionPacketReceiver::addPacketToBuffer(const VisionPacket & packet) -> void
{
  packet_buffer_.push(packet);

  // Maintain buffer size limit
  while (packet_buffer_.size() > max_buffer_size_) {
    packet_buffer_.pop();
    dropped_packet_count_++;
  }
}

auto VisionPacketReceiver::clearOldPackets() -> void
{
  // Remove packets older than 5 seconds from buffer
  const double max_age = 5.0;
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();

  std::queue<VisionPacket> new_buffer;
  while (!packet_buffer_.empty()) {
    const auto & packet = packet_buffer_.front();
    double age = (now - packet.metadata.t_received).seconds();

    if (age <= max_age) {
      new_buffer.push(packet);
    }
    packet_buffer_.pop();
  }

  packet_buffer_ = std::move(new_buffer);
}

auto VisionPacketReceiver::diagnoseNetworkIssues() -> std::string
{
  std::string diagnosis = "Network diagnosis: ";

  if (!checkNetworkInterface()) {
    diagnosis += "Network interface issue detected. ";
  }

  if (packet_rate_ < expected_packet_rate_ * 0.1) {
    diagnosis += "Very low packet rate - check network connection. ";
  }

  if (getConnectionQuality() < 0.3) {
    diagnosis += "Poor connection quality - consider checking firewall/network settings. ";
  }

  return diagnosis;
}

auto VisionPacketReceiver::checkNetworkInterface() const -> bool
{
  // Simplified network interface check
  // In a real implementation, this would check if the multicast interface is available
  return receiver_ != nullptr;
}

// VisionReceiverMonitor implementation
VisionReceiverMonitor::VisionReceiverMonitor(rclcpp::Node & node)
: node_(node), monitoring_active_(false)
{
}

auto VisionReceiverMonitor::addReceiver(
  const std::string & name, std::shared_ptr<VisionPacketReceiver> receiver) -> void
{
  receivers_[name] = receiver;
}

auto VisionReceiverMonitor::removeReceiver(const std::string & name) -> void
{
  receivers_.erase(name);
}

auto VisionReceiverMonitor::getReceiver(const std::string & name) const
  -> std::shared_ptr<VisionPacketReceiver>
{
  auto it = receivers_.find(name);
  return (it != receivers_.end()) ? it->second : nullptr;
}

auto VisionReceiverMonitor::startMonitoring(double check_interval_seconds) -> void
{
  if (monitoring_active_) {
    return;
  }

  monitoring_active_ = true;
  auto interval = std::chrono::duration<double>(check_interval_seconds);

  monitoring_timer_ = node_.create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(interval),
    std::bind(&VisionReceiverMonitor::performHealthCheck, this));
}

auto VisionReceiverMonitor::stopMonitoring() -> void
{
  monitoring_active_ = false;
  monitoring_timer_.reset();
}

auto VisionReceiverMonitor::getOverallHealth() const -> double
{
  if (receivers_.empty()) {
    return 0.0;
  }

  double total_health = 0.0;
  size_t active_count = 0;

  for (const auto & pair : receivers_) {
    double health = pair.second->getConnectionQuality();
    if (health > 0.0) {
      total_health += health;
      active_count++;
    }
  }

  return active_count > 0 ? total_health / active_count : 0.0;
}

auto VisionReceiverMonitor::getHealthReport() const -> std::string
{
  std::ostringstream report;
  report << "Vision Receiver Health Report:\n";
  report << "Overall Health: " << (getOverallHealth() * 100.0) << "%\n";
  report << "Active Receivers: " << getActiveReceiverCount() << "/" << receivers_.size() << "\n";

  for (const auto & pair : receivers_) {
    const auto & name = pair.first;
    const auto & receiver = pair.second;

    report << "  " << name << ": ";
    report << "Quality=" << (receiver->getConnectionQuality() * 100.0) << "%, ";
    report << "Rate=" << receiver->getPacketRate() << "Hz, ";
    report << "Status=" << static_cast<int>(receiver->getStatus()) << "\n";
  }

  return report.str();
}

auto VisionReceiverMonitor::getActiveReceiverCount() const -> size_t
{
  size_t count = 0;
  for (const auto & pair : receivers_) {
    if (pair.second->isActive()) {
      count++;
    }
  }
  return count;
}

auto VisionReceiverMonitor::performHealthCheck() -> void
{
  generateHealthReport();

  if (health_callback_) {
    double overall_health = getOverallHealth();
    std::string status_summary = getHealthReport();
    health_callback_(status_summary, overall_health);
  }
}

auto VisionReceiverMonitor::generateHealthReport() -> void
{
  // Process each receiver and trigger any necessary actions
  for (const auto & pair : receivers_) {
    const auto & receiver = pair.second;
    receiver->processIncomingPackets();
  }
}
}  // namespace crane
