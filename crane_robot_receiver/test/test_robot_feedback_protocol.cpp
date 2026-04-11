// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <array>

#include "crane_robot_receiver/robot_feedback_protocol.hpp"

namespace protocol = crane::robot_receiver::protocol;

namespace
{
auto makeValidPacket() -> std::array<uint8_t, protocol::PACKET_SIZE>
{
  std::array<uint8_t, protocol::PACKET_SIZE> packet{};
  packet[protocol::offset::SYNC_0] = protocol::SYNC_0_VALUE;
  packet[protocol::offset::SYNC_1] = protocol::SYNC_1_VALUE;
  packet[protocol::offset::COUNTER] = 42;
  packet[protocol::offset::CAMERA_FPS] = 30;
  packet[protocol::offset::MOTOR_CURRENT_0] = 12;
  packet[protocol::offset::CHECKSUM] = protocol::computeChecksum(packet);
  return packet;
}
}  // namespace

TEST(RobotFeedbackProtocolTest, ValidPacketPassesValidation)
{
  const auto packet = makeValidPacket();
  const auto result = protocol::validatePacket(packet);

  EXPECT_TRUE(result.size_valid);
  EXPECT_TRUE(result.sync_valid);
  EXPECT_TRUE(result.checksum_valid);
  EXPECT_TRUE(result.valid());
}

TEST(RobotFeedbackProtocolTest, InvalidChecksumIsRejected)
{
  auto packet = makeValidPacket();
  ++packet[protocol::offset::CHECKSUM];

  const auto result = protocol::validatePacket(packet);

  EXPECT_TRUE(result.size_valid);
  EXPECT_TRUE(result.sync_valid);
  EXPECT_FALSE(result.checksum_valid);
  EXPECT_FALSE(result.valid());
}

TEST(RobotFeedbackProtocolTest, InvalidSyncIsRejected)
{
  auto packet = makeValidPacket();
  packet[protocol::offset::SYNC_0] = 0x00;

  const auto result = protocol::validatePacket(packet);

  EXPECT_TRUE(result.size_valid);
  EXPECT_FALSE(result.sync_valid);
  EXPECT_FALSE(result.valid());
}

TEST(RobotFeedbackProtocolTest, InvalidSizeIsRejected)
{
  const std::array<uint8_t, protocol::PACKET_SIZE - 1> packet{};

  const auto result = protocol::validatePacket(packet);

  EXPECT_FALSE(result.size_valid);
  EXPECT_FALSE(result.valid());
}
