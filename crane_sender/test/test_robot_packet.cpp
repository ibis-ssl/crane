// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sender/robot_packet.h>
#include <gtest/gtest.h>

#include <random>

TEST(RobotPacket, EncodeDecode)
{
  std::mt19937 gen;
  std::uniform_real_distribution<float> dist_7(-7.0, 7.0);
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
