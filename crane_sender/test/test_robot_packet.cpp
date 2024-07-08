// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sender/robot_packet.h>
#include <gtest/gtest.h>

#include <random>

TEST(RobotPacket, ENcodeDecode)
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

  const float MAX_ERROR_7 = 7.0 * 2.0 / 32767.0;
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
  packet.lift_dribbler = static_cast<bool>(dist_0_1_int(gen));
  packet.stop_emergency = static_cast<bool>(dist_0_1_int(gen));
  packet.speed_limit = dist_32(gen);
  packet.omega_limit = dist_32(gen);
  packet.latency_time_ms = dist_uint16(gen);
  packet.prioritize_move = static_cast<bool>(dist_0_1_int(gen));
  packet.prioritize_accurate_acceleration = static_cast<bool>(dist_0_1_int(gen));

  {
    // LocalCameraModeArgs
    packet.control_mode = LOCAL_CAMERA_MODE;
    packet.mode_args.local_camera.ball_pos[0] = dist_32(gen);
    packet.mode_args.local_camera.ball_pos[1] = dist_32(gen);
    packet.mode_args.local_camera.ball_vel[0] = dist_32(gen);
    packet.mode_args.local_camera.ball_vel[1] = dist_32(gen);
    packet.mode_args.local_camera.target_global_vel[0] = dist_32(gen);
    packet.mode_args.local_camera.target_global_vel[1] = dist_32(gen);

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
    EXPECT_EQ(packet.lift_dribbler, deserialized_packet.lift_dribbler);
    EXPECT_EQ(packet.stop_emergency, deserialized_packet.stop_emergency);
    EXPECT_NEAR(packet.speed_limit, deserialized_packet.speed_limit, MAX_ERROR_32);
    EXPECT_NEAR(packet.omega_limit, deserialized_packet.omega_limit, MAX_ERROR_32);
    EXPECT_EQ(packet.latency_time_ms, deserialized_packet.latency_time_ms);
    EXPECT_EQ(packet.prioritize_move, deserialized_packet.prioritize_move);
    EXPECT_EQ(
      packet.prioritize_accurate_acceleration,
      deserialized_packet.prioritize_accurate_acceleration);
    EXPECT_EQ(packet.control_mode, deserialized_packet.control_mode);
    EXPECT_NEAR(
      packet.mode_args.local_camera.ball_pos[0],
      deserialized_packet.mode_args.local_camera.ball_pos[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.local_camera.ball_pos[0],
      deserialized_packet.mode_args.local_camera.ball_pos[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.local_camera.ball_vel[0],
      deserialized_packet.mode_args.local_camera.ball_vel[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.local_camera.ball_vel[1],
      deserialized_packet.mode_args.local_camera.ball_vel[1], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.local_camera.target_global_vel[0],
      deserialized_packet.mode_args.local_camera.target_global_vel[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.local_camera.target_global_vel[1],
      deserialized_packet.mode_args.local_camera.target_global_vel[1], MAX_ERROR_32);
  }

  {
    // PositionTargetModeArgs
    packet.control_mode = POSITION_TARGET_MODE;
    packet.mode_args.position.target_global_pos[0] = dist_32(gen);
    packet.mode_args.position.target_global_pos[1] = dist_32(gen);
    packet.mode_args.position.speed_limit_at_target = dist_32(gen);

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
    EXPECT_EQ(packet.lift_dribbler, deserialized_packet.lift_dribbler);
    EXPECT_EQ(packet.stop_emergency, deserialized_packet.stop_emergency);
    EXPECT_NEAR(packet.speed_limit, deserialized_packet.speed_limit, MAX_ERROR_32);
    EXPECT_NEAR(packet.omega_limit, deserialized_packet.omega_limit, MAX_ERROR_32);
    EXPECT_EQ(packet.latency_time_ms, deserialized_packet.latency_time_ms);
    EXPECT_EQ(packet.prioritize_move, deserialized_packet.prioritize_move);
    EXPECT_EQ(
      packet.prioritize_accurate_acceleration,
      deserialized_packet.prioritize_accurate_acceleration);
    EXPECT_EQ(packet.control_mode, deserialized_packet.control_mode);
    EXPECT_NEAR(
      packet.mode_args.position.target_global_pos[0],
      deserialized_packet.mode_args.position.target_global_pos[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.position.target_global_pos[1],
      deserialized_packet.mode_args.position.target_global_pos[1], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.position.speed_limit_at_target,
      deserialized_packet.mode_args.position.speed_limit_at_target, MAX_ERROR_32);
  }

  {
    // SimpleVelocityTargetModeArgs
    packet.control_mode = SIMPLE_VELOCITY_TARGET_MODE;
    packet.mode_args.simple_velocity.target_global_vel[0] = dist_32(gen);
    packet.mode_args.simple_velocity.target_global_vel[1] = dist_32(gen);

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
    EXPECT_EQ(packet.lift_dribbler, deserialized_packet.lift_dribbler);
    EXPECT_EQ(packet.stop_emergency, deserialized_packet.stop_emergency);
    EXPECT_NEAR(packet.speed_limit, deserialized_packet.speed_limit, MAX_ERROR_32);
    EXPECT_NEAR(packet.omega_limit, deserialized_packet.omega_limit, MAX_ERROR_32);
    EXPECT_EQ(packet.latency_time_ms, deserialized_packet.latency_time_ms);
    EXPECT_EQ(packet.prioritize_move, deserialized_packet.prioritize_move);
    EXPECT_EQ(
      packet.prioritize_accurate_acceleration,
      deserialized_packet.prioritize_accurate_acceleration);
    EXPECT_EQ(packet.control_mode, deserialized_packet.control_mode);
    EXPECT_NEAR(
      packet.mode_args.simple_velocity.target_global_vel[0],
      deserialized_packet.mode_args.simple_velocity.target_global_vel[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.simple_velocity.target_global_vel[1],
      deserialized_packet.mode_args.simple_velocity.target_global_vel[1], MAX_ERROR_32);
  }

  {
    // VelocityTargetWithTrajectoryModeArgs
    packet.control_mode = VELOCITY_TARGET_WITH_TRAJECTORY_MODE;
    packet.mode_args.velocity.target_global_vel[0] = dist_32(gen);
    packet.mode_args.velocity.target_global_vel[1] = dist_32(gen);
    packet.mode_args.velocity.trajectory_global_origin[0] = dist_32(gen);
    packet.mode_args.velocity.trajectory_global_origin[1] = dist_32(gen);
    packet.mode_args.velocity.trajectory_origin_angle = dist_pi(gen);
    packet.mode_args.velocity.trajectory_curvature = dist_32(gen);

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
    EXPECT_EQ(packet.lift_dribbler, deserialized_packet.lift_dribbler);
    EXPECT_EQ(packet.stop_emergency, deserialized_packet.stop_emergency);
    EXPECT_NEAR(packet.speed_limit, deserialized_packet.speed_limit, MAX_ERROR_32);
    EXPECT_NEAR(packet.omega_limit, deserialized_packet.omega_limit, MAX_ERROR_32);
    EXPECT_EQ(packet.latency_time_ms, deserialized_packet.latency_time_ms);
    EXPECT_EQ(packet.prioritize_move, deserialized_packet.prioritize_move);
    EXPECT_EQ(
      packet.prioritize_accurate_acceleration,
      deserialized_packet.prioritize_accurate_acceleration);
    EXPECT_EQ(packet.control_mode, deserialized_packet.control_mode);
    EXPECT_NEAR(
      packet.mode_args.velocity.target_global_vel[0],
      deserialized_packet.mode_args.velocity.target_global_vel[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.velocity.target_global_vel[1],
      deserialized_packet.mode_args.velocity.target_global_vel[1], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.velocity.trajectory_global_origin[0],
      deserialized_packet.mode_args.velocity.trajectory_global_origin[0], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.velocity.trajectory_global_origin[1],
      deserialized_packet.mode_args.velocity.trajectory_global_origin[1], MAX_ERROR_32);
    EXPECT_NEAR(
      packet.mode_args.velocity.trajectory_origin_angle,
      deserialized_packet.mode_args.velocity.trajectory_origin_angle, MAX_ERROR_PI);
    EXPECT_NEAR(
      packet.mode_args.velocity.trajectory_curvature,
      deserialized_packet.mode_args.velocity.trajectory_curvature, MAX_ERROR_32);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
