// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_sender/robot_packet.hpp>
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

  const float MAX_ERROR_7 = 7.0 * 2.0 / 32767.0;
  const float MAX_ERROR_32 = 32.0 * 2.0 / 32767.0;
  const float MAX_ERROR_PI = M_PI * 2.0 / 32767.0;
  const float MAX_ERROR_0_1 = 1.0 / 20.0;

  RobotCommand packet;
  packet.HEADER = 4;
  packet.CHECK = 179;
  packet.VEL_LOCAL_SURGE = dist_7(gen);
  packet.VEL_LOCAL_SWAY = dist_7(gen);
  packet.VISION_GLOBAL_X = dist_32(gen);
  packet.VISION_GLOBAL_Y = dist_32(gen);
  packet.VISION_GLOBAL_THETA = dist_pi(gen);
  packet.TARGET_GLOBAL_X = dist_32(gen);
  packet.TARGET_GLOBAL_Y = dist_32(gen);
  packet.BALL_GLOBAL_X = dist_32(gen);
  packet.BALL_GLOBAL_Y = dist_32(gen);
  packet.TARGET_GLOBAL_THETA = dist_pi(gen);
  packet.LOCAL_FEEDBACK_ENABLE = static_cast<bool>(dist_0_1_int(gen));
  packet.LOCAL_KEEPER_MODE_ENABLE = static_cast<bool>(dist_0_1_int(gen));
  packet.IS_ID_VISIBLE = static_cast<bool>(dist_0_1_int(gen));
  packet.STOP_FLAG = static_cast<bool>(dist_0_1_int(gen));
  packet.IS_DRIBBLER_UP = static_cast<bool>(dist_0_1_int(gen));
  packet.KICK_POWER = dist_0_1(gen);
  packet.DRIBBLE_POWER = dist_0_1(gen);
  packet.CHIP_ENABLE = static_cast<bool>(dist_0_1_int(gen));

  RobotCommandSerialized serialized_packet(packet);
  RobotCommand deserialized_packet(serialized_packet);
  //  EXPECT_EQ(packet.HEADER, deserialized_packet.HEADER);
  EXPECT_EQ(packet.CHECK, deserialized_packet.CHECK);
  EXPECT_NEAR(packet.VEL_LOCAL_SURGE, deserialized_packet.VEL_LOCAL_SURGE, MAX_ERROR_7);
  EXPECT_NEAR(packet.VEL_LOCAL_SWAY, deserialized_packet.VEL_LOCAL_SWAY, MAX_ERROR_7);
  EXPECT_NEAR(packet.VISION_GLOBAL_X, deserialized_packet.VISION_GLOBAL_X, MAX_ERROR_32);
  EXPECT_NEAR(packet.VISION_GLOBAL_Y, deserialized_packet.VISION_GLOBAL_Y, MAX_ERROR_32);
  EXPECT_NEAR(packet.VISION_GLOBAL_THETA, deserialized_packet.VISION_GLOBAL_THETA, MAX_ERROR_PI);
  EXPECT_NEAR(packet.TARGET_GLOBAL_X, deserialized_packet.TARGET_GLOBAL_X, MAX_ERROR_32);
  EXPECT_NEAR(packet.TARGET_GLOBAL_Y, deserialized_packet.TARGET_GLOBAL_Y, MAX_ERROR_32);
  EXPECT_NEAR(packet.BALL_GLOBAL_X, deserialized_packet.BALL_GLOBAL_X, MAX_ERROR_32);
  EXPECT_NEAR(packet.BALL_GLOBAL_Y, deserialized_packet.BALL_GLOBAL_Y, MAX_ERROR_32);
  EXPECT_NEAR(packet.TARGET_GLOBAL_THETA, deserialized_packet.TARGET_GLOBAL_THETA, MAX_ERROR_PI);
  EXPECT_EQ(packet.LOCAL_FEEDBACK_ENABLE, deserialized_packet.LOCAL_FEEDBACK_ENABLE);
  EXPECT_EQ(packet.LOCAL_KEEPER_MODE_ENABLE, deserialized_packet.LOCAL_KEEPER_MODE_ENABLE);
  EXPECT_EQ(packet.IS_ID_VISIBLE, deserialized_packet.IS_ID_VISIBLE);
  EXPECT_EQ(packet.STOP_FLAG, deserialized_packet.STOP_FLAG);
  EXPECT_EQ(packet.IS_DRIBBLER_UP, deserialized_packet.IS_DRIBBLER_UP);
  EXPECT_NEAR(packet.KICK_POWER, deserialized_packet.KICK_POWER, MAX_ERROR_0_1);
  EXPECT_NEAR(packet.DRIBBLE_POWER, deserialized_packet.DRIBBLE_POWER, MAX_ERROR_0_1);
  EXPECT_EQ(packet.CHIP_ENABLE, deserialized_packet.CHIP_ENABLE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
