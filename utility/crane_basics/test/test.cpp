// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/travel_time.hpp>

// Circleのテスト
TEST(CircleTest, CreateAndMeasure)
{
  Circle circle;
  circle.center << 0, 0;
  circle.radius = 5.0;

  Point point(10, 0);
  double distance = bg::distance(circle, point);

  EXPECT_DOUBLE_EQ(distance, 5.0);
}

// Capsuleのテスト
TEST(CapsuleTest, CreateAndMeasure)
{
  Capsule capsule;
  capsule.segment.first << 0, 0;
  capsule.segment.second << 10, 0;
  capsule.radius = 2.0;

  Point point(5, 5);
  double distance = bg::distance(capsule, point);

  EXPECT_DOUBLE_EQ(distance, 3.0);
}

TEST(TravelTimeTrapezoidalTest, getTravelTimeTrapezoidal_Stop_NoCruise)
{
  auto stopped_robot = std::make_shared<crane::RobotInfo>();
  stopped_robot->pose.pos << 0, 0;
  stopped_robot->pose.theta = 0;
  stopped_robot->vel.linear << 0, 0;

  Point target;
  target << 4, 0;

  // 加速度1m/s^2, 最高速度4m/s
  // 2秒加速(0~2m/s, 2m)
  // 2秒減速(2~0m/s, 2m)
  // 期待出力時間: 4.0(4m進む)
  double time = crane::getTravelTimeTrapezoidal(stopped_robot, target, 1., 4.);

  EXPECT_DOUBLE_EQ(time, 4.0);
}

TEST(TravelTimeTrapezoidalTest, getTravelTimeTrapezoidal_Stop_Cruise)
{
  auto stopped_robot = std::make_shared<crane::RobotInfo>();
  stopped_robot->pose.pos << 0, 0;
  stopped_robot->pose.theta = 0;
  stopped_robot->vel.linear << 0, 0;

  Point target;
  target << 8, 0;
  // 加速度1m/s^2, 最高速度2m/s
  // 2秒加速(0~2m/s, 2m)
  // 2秒等速(2m/s, 4m)
  // 2秒減速(2~0m/s, 2m)
  // 期待出力時間: 6.0(8m進む)
  double time = crane::getTravelTimeTrapezoidal(stopped_robot, target, 1., 2.);

  EXPECT_DOUBLE_EQ(time, 6.0);
}

TEST(TravelTimeTrapezoidalTest, getTravelTimeTrapezoidal_Moving_NoCruise)
{
  auto stopped_robot = std::make_shared<crane::RobotInfo>();
  stopped_robot->pose.pos << 0, 0;
  stopped_robot->pose.theta = 0;
  stopped_robot->vel.linear << 1, 0;

  Point target;
  target << 3.5, 0;

  // 加速度1m/s^2, 最高速度4m/s
  // 1秒加速(1~2m/s, 1.5m): 2^2 - 1^2 = 2 * 1 * x, 3 = 2x, x = 1.5
  // 2秒減速(2~0m/s, 2m): 2^2 - 0^2 = 2 * 1 * x, 4 = 2x, x = 2
  // 期待出力時間: 3.0(3.5m進む)
  double time = crane::getTravelTimeTrapezoidal(stopped_robot, target, 1., 4.);
  EXPECT_DOUBLE_EQ(time, 3.0);
}

TEST(TravelTimeTrapezoidalTest, getTravelTimeTrapezoidal_Moving_Cruise)
{
  auto stopped_robot = std::make_shared<crane::RobotInfo>();
  stopped_robot->pose.pos << 0, 0;
  stopped_robot->pose.theta = 0;
  stopped_robot->vel.linear << 1, 0;

  Point target;
  target << 7.5, 0;

  // 加速度1m/s^2, 最高速度2m/s
  // 1秒加速(1~2m/s, 1.5m)
  // 2秒等速(2m/s, 4m)
  // 2秒減速(2~0m/s, 2m)
  // 期待出力時間: 5.0(7.5m進む)
  double time = crane::getTravelTimeTrapezoidal(stopped_robot, target, 1., 2.);
  EXPECT_DOUBLE_EQ(time, 5.0);
}

// メイン関数
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
