// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include "crane_robot_receiver/robot_feedback_protocol.hpp"

namespace protocol = crane::robot_receiver::protocol;

namespace
{
// 全ペイロードの確定後に byte 2 を書く
void finalizeCrc(std::array<uint8_t, protocol::PACKET_SIZE> & packet)
{
  packet[protocol::offset::CRC] =
    protocol::crc8(packet, protocol::offset::COUNTER, protocol::PACKET_SIZE);
}

auto makeValidPacket() -> std::array<uint8_t, protocol::PACKET_SIZE>
{
  std::array<uint8_t, protocol::PACKET_SIZE> packet{};
  packet[protocol::offset::SYNC_0] = protocol::SYNC_0_VALUE;
  packet[protocol::offset::SYNC_1] = protocol::SYNC_1_VALUE;
  packet[protocol::offset::COUNTER] = 42;
  packet[protocol::offset::MOTOR_CURRENT_0] = 12;
  finalizeCrc(packet);
  return packet;
}
}  // namespace

TEST(RobotFeedbackProtocolTest, ValidPacketPassesValidation)
{
  const auto packet = makeValidPacket();
  const auto result = protocol::validatePacket(packet, packet.size());

  EXPECT_TRUE(result.size_valid);
  EXPECT_TRUE(result.sync_valid);
  EXPECT_TRUE(result.valid());
}

TEST(RobotFeedbackProtocolTest, InvalidSyncIsRejected)
{
  auto packet = makeValidPacket();
  packet[protocol::offset::SYNC_0] = 0x00;

  const auto result = protocol::validatePacket(packet, packet.size());

  EXPECT_TRUE(result.size_valid);
  EXPECT_FALSE(result.sync_valid);
  EXPECT_FALSE(result.valid());
}

TEST(RobotFeedbackProtocolTest, InvalidSizeIsRejected)
{
  const auto packet = makeValidPacket();

  const auto result = protocol::validatePacket(packet, protocol::PACKET_SIZE - 1);

  EXPECT_FALSE(result.size_valid);
  EXPECT_FALSE(result.valid());
}

// 実際の受信経路では BUFFER_SIZE (2048) の使い回しバッファに 128 バイトが届く。
// buffer.size() をパケット長として見ると、この形が常に size_valid=false になる。
TEST(RobotFeedbackProtocolTest, ReceiveBufferLargerThanPacketIsAccepted)
{
  std::vector<char> buffer(protocol::BUFFER_SIZE, 0);
  const auto packet = makeValidPacket();
  for (size_t i = 0; i < packet.size(); ++i) {
    buffer[i] = static_cast<char>(packet[i]);
  }

  const auto result = protocol::validatePacket(buffer, protocol::PACKET_SIZE);

  EXPECT_TRUE(result.size_valid);
  EXPECT_TRUE(result.sync_valid);
  EXPECT_TRUE(result.valid());
}

TEST(RobotFeedbackProtocolTest, Crc8MatchesStandardCheckValue)
{
  const std::string check = "123456789";
  EXPECT_EQ(protocol::crc8(check, 0, check.size()), 0xF4);
}

TEST(RobotFeedbackProtocolTest, CorruptedPayloadIsRejectedByCrc)
{
  auto packet = makeValidPacket();
  packet[protocol::offset::MOTOR_CURRENT_0] ^= 0x01;

  const auto result = protocol::validatePacket(packet, packet.size());

  EXPECT_TRUE(result.size_valid);
  EXPECT_TRUE(result.sync_valid);
  EXPECT_FALSE(result.crc_valid);
  EXPECT_FALSE(result.valid());
}

// 同期バイトは CRC の対象外。同期バイトだけが違うパケットは sync で弾かれ、CRC は有効のまま。
TEST(RobotFeedbackProtocolTest, SyncBytesAreOutsideCrcCoverage)
{
  auto packet = makeValidPacket();
  packet[protocol::offset::SYNC_1] = 0xEB;

  const auto result = protocol::validatePacket(packet, packet.size());

  EXPECT_FALSE(result.sync_valid);
  EXPECT_TRUE(result.crc_valid);
}
