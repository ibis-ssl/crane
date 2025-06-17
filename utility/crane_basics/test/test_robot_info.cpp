// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/robot_info.hpp>

namespace crane
{
// RobotInfoクラスのテスト
TEST(RobotInfoTest, BasicOperations)
{
  RobotInfo robot;
  robot.id = 1;
  robot.pose.pos << 2.0, 3.0;
  robot.pose.theta = M_PI / 4.0;  // 45度
  robot.vel.linear << 1.0, 0.0;
  robot.vel.omega = 0.1;

  // getIDのテスト
  auto id = robot.getID();
  EXPECT_TRUE(id.is_ours);
  EXPECT_EQ(id.id, 1);

  // center_to_kickerのテスト
  Vector2 kicker_vec = robot.center_to_kicker();
  // 45度の方向に0.090m
  EXPECT_NEAR(kicker_vec.x(), 0.090 * cos(M_PI / 4.0), 1e-10);
  EXPECT_NEAR(kicker_vec.y(), 0.090 * sin(M_PI / 4.0), 1e-10);

  // kicker_centerのテスト
  Point kicker_center = robot.kicker_center();
  EXPECT_NEAR(kicker_center.x(), 2.0 + 0.090 * cos(M_PI / 4.0), 1e-10);
  EXPECT_NEAR(kicker_center.y(), 3.0 + 0.090 * sin(M_PI / 4.0), 1e-10);

  // geometryのテスト
  auto geom = robot.geometry();
  EXPECT_DOUBLE_EQ(geom.radius, 0.060);
  EXPECT_DOUBLE_EQ(geom.center.x(), 2.0);
  EXPECT_DOUBLE_EQ(geom.center.y(), 3.0);

  // getDistanceのテスト
  Point test_point;
  test_point << 5.0, 3.0;
  EXPECT_DOUBLE_EQ(robot.getDistance(test_point), 3.0);

  Pose2D test_pose;
  test_pose.pos << 2.0, 4.0;
  EXPECT_DOUBLE_EQ(robot.getDistance(test_pose), 1.0);
}

// RobotIdentifierのテスト
TEST(RobotIdentifierTest, Comparison)
{
  RobotIdentifier id1{true, 1};
  RobotIdentifier id2{true, 1};
  RobotIdentifier id3{true, 2};
  RobotIdentifier id4{false, 1};

  // 等価テスト
  EXPECT_TRUE(id1 == id2);
  EXPECT_FALSE(id1 == id3);
  EXPECT_FALSE(id1 == id4);

  // 非等価テスト
  EXPECT_FALSE(id1 != id2);
  EXPECT_TRUE(id1 != id3);
  EXPECT_TRUE(id1 != id4);
}
}  // namespace crane
