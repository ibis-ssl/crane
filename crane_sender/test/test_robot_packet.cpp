// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sender/robot_packet.h>
#include <gtest/gtest.h>

#include <cmath>
#include <crane_sender/latency_time.hpp>
#include <random>
#include <utility>

TEST(RobotPacket, EncodeDecode)
{
  std::mt19937 gen;
  std::uniform_real_distribution<float> dist_32(-32, 32);
  std::uniform_real_distribution<float> dist_pi(-M_PI, M_PI);
  std::uniform_real_distribution<float> dist_0_1(0.0, 1.0);
  // 0 or 1
  std::uniform_int_distribution<int> dist_0_1_int(0, 1);
  // uint16
  std::uniform_int_distribution<uint16_t> dist_uint16(0, 65535);

  const float MAX_ERROR_32 = 32.0 * 2.0 / 32767.0;
  const float MAX_ERROR_PI = M_PI * 2.0 / 32767.0;
  const float MAX_ERROR_0_1 = 1.0 / 20.0;

  RobotCommandV2 packet;
  packet.header = 4;
  packet.check_counter = 179;
  packet.vision_global_pos[0] = dist_32(gen);
  packet.vision_global_pos[1] = dist_32(gen);
  packet.vision_global_theta = dist_pi(gen);
  packet.is_vision_available = static_cast<bool>(dist_0_1_int(gen));
  packet.target_global_theta = dist_pi(gen);
  packet.kick_power = dist_0_1(gen);
  packet.dribble_power = dist_0_1(gen);
  packet.enable_chip = static_cast<bool>(dist_0_1_int(gen));
  packet.stop_emergency = static_cast<bool>(dist_0_1_int(gen));
  packet.acceleration_limit = dist_32(gen);
  packet.linear_velocity_limit = dist_32(gen);
  packet.angular_velocity_limit = dist_32(gen);
  packet.latency_time_ms = dist_uint16(gen);
  packet.elapsed_time_ms_since_last_vision = dist_uint16(gen);

  {
    packet.target_global_pos[0] = dist_32(gen);
    packet.target_global_pos[1] = dist_32(gen);
    packet.terminal_velocity = dist_32(gen);

    packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
    packet.mode_args.polar_velocity.target_global_velocity_r = dist_32(gen);
    packet.mode_args.polar_velocity.target_global_velocity_theta = dist_32(gen);

    RobotCommandSerializedV2 serialized_packet;
    RobotCommandSerializedV2_serialize(&serialized_packet, &packet);

    RobotCommandV2 deserialized_packet = RobotCommandSerializedV2_deserialize(&serialized_packet);
    EXPECT_EQ(packet.header, deserialized_packet.header);
    EXPECT_EQ(packet.check_counter, deserialized_packet.check_counter);
    EXPECT_NEAR(
      packet.vision_global_pos[0], deserialized_packet.vision_global_pos[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.vision_global_pos[1], deserialized_packet.vision_global_pos[1], MAX_ERROR_32);
    EXPECT_NEAR(packet.vision_global_theta, deserialized_packet.vision_global_theta, MAX_ERROR_PI);
    EXPECT_EQ(packet.is_vision_available, deserialized_packet.is_vision_available);
    EXPECT_NEAR(packet.target_global_theta, deserialized_packet.target_global_theta, MAX_ERROR_PI);
    EXPECT_NEAR(packet.kick_power, deserialized_packet.kick_power, MAX_ERROR_0_1);
    EXPECT_NEAR(packet.dribble_power, deserialized_packet.dribble_power, MAX_ERROR_0_1);
    EXPECT_EQ(packet.enable_chip, deserialized_packet.enable_chip);
    EXPECT_EQ(packet.stop_emergency, deserialized_packet.stop_emergency);
    EXPECT_NEAR(packet.acceleration_limit, deserialized_packet.acceleration_limit, MAX_ERROR_32);
    EXPECT_NEAR(
      packet.linear_velocity_limit, deserialized_packet.linear_velocity_limit, MAX_ERROR_32);
    EXPECT_NEAR(
      packet.angular_velocity_limit, deserialized_packet.angular_velocity_limit, MAX_ERROR_32);
    EXPECT_EQ(packet.latency_time_ms, deserialized_packet.latency_time_ms);
    EXPECT_EQ(
      packet.elapsed_time_ms_since_last_vision,
      deserialized_packet.elapsed_time_ms_since_last_vision);
    EXPECT_EQ(packet.control_mode, deserialized_packet.control_mode);
    EXPECT_NEAR(
      packet.mode_args.polar_velocity.target_global_velocity_r,
      deserialized_packet.mode_args.polar_velocity.target_global_velocity_r, MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.polar_velocity.target_global_velocity_theta,
      deserialized_packet.mode_args.polar_velocity.target_global_velocity_theta, MAX_ERROR_32);
    EXPECT_NEAR(
      packet.target_global_pos[0], deserialized_packet.target_global_pos[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.target_global_pos[1], deserialized_packet.target_global_pos[1], MAX_ERROR_32);
    EXPECT_NEAR(packet.terminal_velocity, deserialized_packet.terminal_velocity, MAX_ERROR_32);
  }

  {
    packet.target_global_pos[0] = dist_32(gen);
    packet.target_global_pos[1] = dist_32(gen);
    packet.terminal_velocity = dist_32(gen);

    packet.control_mode = POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE;
    packet.mode_args.position_target.terminal_velocity_x = dist_32(gen);
    packet.mode_args.position_target.terminal_velocity_y = dist_32(gen);

    RobotCommandSerializedV2 serialized_packet;
    RobotCommandSerializedV2_serialize(&serialized_packet, &packet);

    RobotCommandV2 deserialized_packet = RobotCommandSerializedV2_deserialize(&serialized_packet);
    EXPECT_EQ(packet.control_mode, deserialized_packet.control_mode);
    EXPECT_NEAR(
      packet.mode_args.position_target.terminal_velocity_x,
      deserialized_packet.mode_args.position_target.terminal_velocity_x, MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.position_target.terminal_velocity_y,
      deserialized_packet.mode_args.position_target.terminal_velocity_y, MAX_ERROR_32);
    EXPECT_NEAR(
      packet.target_global_pos[0], deserialized_packet.target_global_pos[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.target_global_pos[1], deserialized_packet.target_global_pos[1], MAX_ERROR_32);
    EXPECT_NEAR(packet.terminal_velocity, deserialized_packet.terminal_velocity, MAX_ERROR_32);
  }
}

// mode 4 のバイト境界を固定するテスト。
//
// 上の EncodeDecode は serialize -> deserialize の「対称性」しか見ていないため、
// 両者が同じ誤ったオフセットを使っていても通ってしまう。robot_packet.h は
// crane / G474_Orion_main / framework の 3 リポジトリで共有される正本であり、
// 守るべきは絶対オフセットそのものなので、ここで直接固定する。
TEST(RobotPacket, Mode4ByteLayout)
{
  const float MAX_ERROR_32 = 32.0 * 2.0 / 32767.0;

  // オフセット自体を固定する（enum の並び替えや項目追加による無言のズレを検出する）
  EXPECT_EQ(23, static_cast<int>(CONTROL_MODE));
  EXPECT_EQ(24, static_cast<int>(CONTROL_MODE_ARGS));
  EXPECT_EQ(32, static_cast<int>(TARGET_GLOBAL_POS_X_HIGH));
  EXPECT_EQ(34, static_cast<int>(TARGET_GLOBAL_POS_Y_HIGH));
  EXPECT_EQ(36, static_cast<int>(TERMINAL_VELOCITY_HIGH));

  RobotCommandV2 packet{};
  packet.header = 0x00;
  packet.check_counter = 1;
  packet.control_mode = POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE;
  packet.mode_args.position_target.terminal_velocity_x = 1.25;
  packet.mode_args.position_target.terminal_velocity_y = -0.75;
  packet.target_global_pos[0] = 2.5;
  packet.target_global_pos[1] = -1.5;
  packet.terminal_velocity = 0.5;

  RobotCommandSerializedV2 serialized;
  RobotCommandSerializedV2_serialize(&serialized, &packet);

  // byte 23 == 4
  EXPECT_EQ(POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE, serialized.data[23]);

  // byte 24..27 = terminal_velocity_x / y（mode 4 の ARGS）
  EXPECT_NEAR(
    1.25, convertTwoByteToFloat(serialized.data[24], serialized.data[25], 32.767), MAX_ERROR_32);
  EXPECT_NEAR(
    -0.75, convertTwoByteToFloat(serialized.data[26], serialized.data[27], 32.767), MAX_ERROR_32);

  // byte 32..37 = TARGET_GLOBAL_POS_X / Y, TERMINAL_VELOCITY（mode に依存しない固定フィールド）
  EXPECT_NEAR(
    2.5, convertTwoByteToFloat(serialized.data[32], serialized.data[33], 32.767), MAX_ERROR_32);
  EXPECT_NEAR(
    -1.5, convertTwoByteToFloat(serialized.data[34], serialized.data[35], 32.767), MAX_ERROR_32);
  EXPECT_NEAR(
    0.5, convertTwoByteToFloat(serialized.data[36], serialized.data[37], 32.767), MAX_ERROR_32);
}

// mode 3 も同じく byte 23 と ARGS の位置を固定する。
TEST(RobotPacket, Mode3ByteLayout)
{
  const float MAX_ERROR_32 = 32.0 * 2.0 / 32767.0;

  RobotCommandV2 packet{};
  packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
  packet.mode_args.polar_velocity.target_global_velocity_r = 1.25;
  packet.mode_args.polar_velocity.target_global_velocity_theta = -0.75;

  RobotCommandSerializedV2 serialized;
  RobotCommandSerializedV2_serialize(&serialized, &packet);

  EXPECT_EQ(POLAR_VELOCITY_TARGET_MODE, serialized.data[23]);
  EXPECT_NEAR(
    1.25, convertTwoByteToFloat(serialized.data[24], serialized.data[25], 32.767), MAX_ERROR_32);
  EXPECT_NEAR(
    -0.75, convertTwoByteToFloat(serialized.data[26], serialized.data[27], 32.767), MAX_ERROR_32);
}

// CONTROL_MODE_ARGS(24..31) は mode によって意味が変わる union である。
//
// このテストは「mode を見ずに復号すると何が起きるか」を実行可能な形で固定する。
// mode 4 のバイト列を mode 3 として読むと、terminal_velocity_x/y が
// そのまま r/theta として解釈される。値としては壊れていないため
// 受信側はチェックサムでもレンジチェックでも誤りに気づけない。
// すなわち CONTROL_MODE を先に見る以外にこの取り違えを防ぐ手段はない。
TEST(RobotPacket, ModeArgsUnionMustNotBeDecodedWithoutControlMode)
{
  const float MAX_ERROR_32 = 32.0 * 2.0 / 32767.0;

  RobotCommandV2 packet{};
  packet.control_mode = POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE;
  packet.mode_args.position_target.terminal_velocity_x = 1.25;
  packet.mode_args.position_target.terminal_velocity_y = -0.75;

  RobotCommandSerializedV2 serialized;
  RobotCommandSerializedV2_serialize(&serialized, &packet);

  // CONTROL_MODE だけを mode 3 に偽装する（= mode を見ない受信側と同じ状況）
  RobotCommandSerializedV2 spoofed = serialized;
  spoofed.data[CONTROL_MODE] = POLAR_VELOCITY_TARGET_MODE;

  const RobotCommandV2 misread = RobotCommandSerializedV2_deserialize(&spoofed);

  EXPECT_EQ(POLAR_VELOCITY_TARGET_MODE, misread.control_mode);
  // 終端速度ベクトルが「極座標の r / theta」として無言で読まれてしまう
  EXPECT_NEAR(1.25, misread.mode_args.polar_velocity.target_global_velocity_r, MAX_ERROR_32);
  EXPECT_NEAR(-0.75, misread.mode_args.polar_velocity.target_global_velocity_theta, MAX_ERROR_32);
}

// latency_ms から送信バイトまで。255 を超える値が 8bit に切り詰められないこと
TEST(RobotPacket, LatencyTimeBytes)
{
  const auto serialize_latency = [](float latency_ms) {
    RobotCommandV2 packet{};
    packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
    packet.latency_time_ms = crane::toLatencyTimeMs(latency_ms);
    RobotCommandSerializedV2 serialized{};
    RobotCommandSerializedV2_serialize(&serialized, &packet);
    return std::make_pair(
      serialized.data[LATENCY_TIME_MS_HIGH], serialized.data[LATENCY_TIME_MS_LOW]);
  };

  EXPECT_EQ(serialize_latency(300.0f), std::make_pair(uint8_t{0x01}, uint8_t{0x2C}));
  EXPECT_EQ(serialize_latency(65535.0f), std::make_pair(uint8_t{0xFF}, uint8_t{0xFF}));
  EXPECT_EQ(serialize_latency(70000.0f), std::make_pair(uint8_t{0xFF}, uint8_t{0xFF}));
  EXPECT_EQ(serialize_latency(-1.0f), std::make_pair(uint8_t{0x00}, uint8_t{0x00}));
  EXPECT_EQ(serialize_latency(NAN), std::make_pair(uint8_t{0x00}, uint8_t{0x00}));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
