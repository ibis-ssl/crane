// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "crane_robot_receiver/robot_feedback_protocol.hpp"

namespace protocol = crane::robot_receiver::protocol;

namespace
{
auto makeValidPacket() -> std::array<uint8_t, protocol::PACKET_SIZE>
{
  std::array<uint8_t, protocol::PACKET_SIZE> packet{};
  packet[protocol::offset::SYNC_0] = protocol::SYNC_0_VALUE;
  packet[protocol::offset::SYNC_1] = protocol::SYNC_1_VALUE;
  // 実機と同じく byte 2 は定数。チェックサムではない。
  packet[protocol::offset::DUMMY_CRC] = protocol::DUMMY_CRC_VALUE;
  packet[protocol::offset::COUNTER] = 42;
  packet[protocol::offset::CAMERA_FPS] = 30;
  packet[protocol::offset::MOTOR_CURRENT_0] = 12;
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

// byte 2 は G474 ファームウェア (ai_comm.c: `buf[2] = 10;  // CRC, 10:dummy`) が
// 定数 10 を書くだけで、チェックサムではない。
// 受信側で byte 2 を検証すると実機パケットが全滅するため、
// 「byte 2 が何であっても検証結果に影響しない」ことを回帰テストとして固定する。
TEST(RobotFeedbackProtocolTest, DummyCrcByteDoesNotAffectValidation)
{
  auto packet = makeValidPacket();
  for (const uint8_t value : {uint8_t{0}, protocol::DUMMY_CRC_VALUE, uint8_t{0xFF}}) {
    packet[protocol::offset::DUMMY_CRC] = value;
    const auto result = protocol::validatePacket(packet, packet.size());
    EXPECT_TRUE(result.valid()) << "byte 2 = " << static_cast<int>(value);
  }
}
