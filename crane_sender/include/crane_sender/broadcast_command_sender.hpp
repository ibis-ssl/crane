// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__BROADCAST_COMMAND_SENDER_HPP_
#define CRANE_SENDER__BROADCAST_COMMAND_SENDER_HPP_

#include <boost/asio.hpp>
#include <cstdint>
#include <utility>
#include <vector>

#include "crane_sender/robot_packet.h"

namespace crane
{
// 通信設定の定数
namespace CommConfig
{
constexpr int DEFAULT_PORT = 12345;
constexpr const char * BROADCAST_ADDRESS = "192.168.20.255";
constexpr int AI_CMD_V3_SIZE = 64;
constexpr int AI_CMD_V3_ROBOT_NUM = 11;
}  // namespace CommConfig

class BroadcastCommandSender
{
public:
  explicit BroadcastCommandSender();

  void sendBroadcastPackets(
    const std::vector<std::pair<uint8_t, RobotCommandSerializedV3>> & robot_packets,
    int check_counter);

private:
  void checkNetworkInterfaces() const;

protected:
  boost::asio::io_service io_service;
  boost::asio::ip::udp::endpoint endpoint;
  boost::asio::ip::udp::socket socket;
};
}  // namespace crane

#endif  // CRANE_SENDER__BROADCAST_COMMAND_SENDER_HPP_
