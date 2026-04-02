// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_sender/broadcast_command_sender.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
BroadcastCommandSender::BroadcastCommandSender(const std::string & address, int port)
: socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
{
  try {
    // ブロードキャスト許可フラグを設定
    socket.set_option(boost::asio::socket_base::broadcast(true));
    RCLCPP_INFO(rclcpp::get_logger("BroadcastCommandSender"), "✓ SO_BROADCAST flag set");

    std::string host = address;
    endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(host), port);

    // インターフェース情報の確認（デバッグ用）
    checkNetworkInterfaces();

    RCLCPP_INFO(
      rclcpp::get_logger("BroadcastCommandSender"), "【Real Robot Broadcast Mode Initialized】");
    RCLCPP_INFO(
      rclcpp::get_logger("BroadcastCommandSender"), "  Target Address: %s:%d", host.c_str(), port);
    RCLCPP_INFO(
      rclcpp::get_logger("BroadcastCommandSender"), "  Resolved Endpoint: %s:%d",
      endpoint.address().to_string().c_str(), endpoint.port());
    RCLCPP_INFO(
      rclcpp::get_logger("BroadcastCommandSender"), "  Local Socket: %s:%d",
      socket.local_endpoint().address().to_string().c_str(), socket.local_endpoint().port());
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BroadcastCommandSender"), "❌ BroadcastCommandSender Init Error: %s",
      e.what());
  }
}

void BroadcastCommandSender::sendBroadcastPackets(
  const std::array<std::pair<uint8_t, RobotCommandSerializedV2>, CommConfig::AI_CMD_V2_ROBOT_NUM> &
    robot_packets,
  [[maybe_unused]] int check_counter)
{
  char broadcast_buf[(CommConfig::AI_CMD_V2_SIZE + 1) * CommConfig::AI_CMD_V2_ROBOT_NUM] = {};

  for (size_t i = 0; i < CommConfig::AI_CMD_V2_ROBOT_NUM; i++) {
    int offset = static_cast<int>(i) * (CommConfig::AI_CMD_V2_SIZE + 1);
    broadcast_buf[offset] = static_cast<char>(i);
    memcpy(&broadcast_buf[offset + 1], robot_packets[i].second.data, CommConfig::AI_CMD_V2_SIZE);
  }

  // パケット送信
  try {
    socket.send_to(boost::asio::buffer(broadcast_buf), endpoint);
  } catch (boost::system::system_error & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BroadcastCommandSender"), "❌ Packet Send Error (boost): %s", e.what());
    RCLCPP_ERROR(
      rclcpp::get_logger("BroadcastCommandSender"), "  Error Code: %d", e.code().value());
    RCLCPP_ERROR(
      rclcpp::get_logger("BroadcastCommandSender"), "  Error Message: %s",
      e.code().message().c_str());
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BroadcastCommandSender"), "❌ Packet Send Exception: %s", e.what());
  }
}

void BroadcastCommandSender::checkNetworkInterfaces() const
{
  struct ifaddrs * interfaces = nullptr;

  RCLCPP_INFO(rclcpp::get_logger("BroadcastCommandSender"), "🌐 Available Network Interfaces:");

  if (getifaddrs(&interfaces) == -1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BroadcastCommandSender"), "❌ Failed to get network interface info");
    return;
  }

  for (struct ifaddrs * ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr) continue;

    // IPv4アドレスのみ表示
    if (ifa->ifa_addr->sa_family == AF_INET) {
      struct sockaddr_in * addr_in = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
      char ip_str[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(addr_in->sin_addr), ip_str, INET_ADDRSTRLEN);

      // ブロードキャストアドレス情報も取得
      char broadcast_str[INET_ADDRSTRLEN] = "N/A";
      if (ifa->ifa_flags & IFF_BROADCAST && ifa->ifa_broadaddr) {
        struct sockaddr_in * broadcast_in =
          reinterpret_cast<struct sockaddr_in *>(ifa->ifa_broadaddr);
        inet_ntop(AF_INET, &(broadcast_in->sin_addr), broadcast_str, INET_ADDRSTRLEN);
      }

      std::string log_msg = "  Interface: " + std::string(ifa->ifa_name) +
                            " IP: " + std::string(ip_str) +
                            " Broadcast: " + std::string(broadcast_str);

      // インターフェースの状態を表示
      if (ifa->ifa_flags & IFF_UP) log_msg += " [UP]";
      if (ifa->ifa_flags & IFF_RUNNING) log_msg += " [RUNNING]";
      if (ifa->ifa_flags & IFF_BROADCAST) log_msg += " [BROADCAST]";

      RCLCPP_INFO(rclcpp::get_logger("BroadcastCommandSender"), "%s", log_msg.c_str());

      // 設定されたブロードキャストアドレスとの照合
      if (std::string(broadcast_str) == CommConfig::BROADCAST_ADDRESS) {
        RCLCPP_INFO(
          rclcpp::get_logger("BroadcastCommandSender"),
          "    ✅ Matches configured broadcast address!");
      }
    }
  }

  freeifaddrs(interfaces);
}
}  // namespace crane
