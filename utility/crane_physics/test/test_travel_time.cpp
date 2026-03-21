// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/robot_info.hpp>
#include <crane_physics/travel_time.hpp>

namespace crane
{
TEST(TravelTimeTrapezoidalTest, getTravelTimeTrapezoidal_Stop_NoCruise)
{
  auto stopped_robot = std::make_shared<crane::RobotInfo>();
  stopped_robot->pose = Pose2D{.pos = Point(0.0, 0.0), .theta = 0.0};
  stopped_robot->vel = Velocity2D{.linear = Point(0.0, 0.0), .omega = 0.0};

  Point target(4.0, 0.0);

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
  stopped_robot->pose = Pose2D{.pos = Point(0.0, 0.0), .theta = 0.0};
  stopped_robot->vel = Velocity2D{.linear = Point(0.0, 0.0), .omega = 0.0};

  Point target(8.0, 0.0);
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
  stopped_robot->pose = Pose2D{.pos = Point(0.0, 0.0), .theta = 0.0};
  stopped_robot->vel = Velocity2D{.linear = Point(1.0, 0.0), .omega = 0.0};

  Point target(3.5, 0.0);

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
  stopped_robot->pose = Pose2D{.pos = Point(0.0, 0.0), .theta = 0.0};
  stopped_robot->vel = Velocity2D{.linear = Point(1.0, 0.0), .omega = 0.0};

  Point target(7.5, 0.0);

  // 加速度1m/s^2, 最高速度2m/s
  // 1秒加速(1~2m/s, 1.5m)
  // 2秒等速(2m/s, 4m)
  // 2秒減速(2~0m/s, 2m)
  // 期待出力時間: 5.0(7.5m進む)
  double time = crane::getTravelTimeTrapezoidal(stopped_robot, target, 1., 2.);
  EXPECT_DOUBLE_EQ(time, 5.0);
}

// getPredictedPositionTrapezoidal のテスト
TEST(PredictedPositionTrapezoidalTest, TimeZero_ReturnsCurrentPosition)
{
  Point current_pos(0.0, 0.0);
  Vector2 current_vel(1.0, 0.0);
  Point target_pos(5.0, 0.0);

  Point predicted =
    crane::getPredictedPositionTrapezoidal(current_pos, current_vel, target_pos, 0.0, 1.0, 2.0);

  EXPECT_DOUBLE_EQ(predicted.x(), 0.0);
  EXPECT_DOUBLE_EQ(predicted.y(), 0.0);
}

TEST(PredictedPositionTrapezoidalTest, TimeExceedsTravelTime_ReturnsTargetPosition)
{
  Point current_pos(0.0, 0.0);
  Vector2 current_vel(0.0, 0.0);
  Point target_pos(4.0, 0.0);

  // 移動時間は4秒（加速2秒+減速2秒、定速なし）
  // 10秒後を予測すると目標位置に到達
  Point predicted =
    crane::getPredictedPositionTrapezoidal(current_pos, current_vel, target_pos, 10.0, 1.0, 4.0);

  EXPECT_DOUBLE_EQ(predicted.x(), 4.0);
  EXPECT_DOUBLE_EQ(predicted.y(), 0.0);
}

TEST(PredictedPositionTrapezoidalTest, AccelerationPhase_Stop_NoCruise)
{
  Point current_pos(0.0, 0.0);
  Vector2 current_vel(0.0, 0.0);
  Point target_pos(4.0, 0.0);

  // 加速度1m/s^2, 最高速度4m/s
  // 2秒加速(0~2m/s, 2m) + 2秒減速(2~0m/s, 2m)
  // 1秒後（加速フェーズ中）: 0.5 * 1 * 1^2 = 0.5m
  Point predicted =
    crane::getPredictedPositionTrapezoidal(current_pos, current_vel, target_pos, 1.0, 1.0, 4.0);

  EXPECT_DOUBLE_EQ(predicted.x(), 0.5);
  EXPECT_DOUBLE_EQ(predicted.y(), 0.0);
}

TEST(PredictedPositionTrapezoidalTest, DecelerationPhase_Stop_NoCruise)
{
  Point current_pos(0.0, 0.0);
  Vector2 current_vel(0.0, 0.0);
  Point target_pos(4.0, 0.0);

  // 加速度1m/s^2, 最高速度4m/s
  // 2秒加速(0~2m/s, 2m) + 2秒減速(2~0m/s, 2m)
  // 3秒後（減速フェーズ中、減速開始から1秒）:
  // 加速: 2m + 減速1秒: 2*1 - 0.5*1*1^2 = 2 - 0.5 = 1.5m
  // 合計: 2 + 1.5 = 3.5m
  Point predicted =
    crane::getPredictedPositionTrapezoidal(current_pos, current_vel, target_pos, 3.0, 1.0, 4.0);

  EXPECT_DOUBLE_EQ(predicted.x(), 3.5);
  EXPECT_DOUBLE_EQ(predicted.y(), 0.0);
}

TEST(PredictedPositionTrapezoidalTest, CruisePhase_Stop_WithCruise)
{
  Point current_pos(0.0, 0.0);
  Vector2 current_vel(0.0, 0.0);
  Point target_pos(8.0, 0.0);

  // 加速度1m/s^2, 最高速度2m/s
  // 2秒加速(0~2m/s, 2m) + 2秒等速(2m/s, 4m) + 2秒減速(2~0m/s, 2m)
  // 3秒後（定速フェーズ中、定速1秒目）:
  // 加速: 2m + 定速1秒: 2*1 = 2m
  // 合計: 2 + 2 = 4m
  Point predicted =
    crane::getPredictedPositionTrapezoidal(current_pos, current_vel, target_pos, 3.0, 1.0, 2.0);

  EXPECT_DOUBLE_EQ(predicted.x(), 4.0);
  EXPECT_DOUBLE_EQ(predicted.y(), 0.0);
}

TEST(PredictedPositionTrapezoidalTest, Moving_NoCruise)
{
  Point current_pos(0.0, 0.0);
  Vector2 current_vel(1.0, 0.0);
  Point target_pos(3.5, 0.0);

  // 加速度1m/s^2, 最高速度4m/s
  // 1秒加速(1~2m/s, 1.5m) + 2秒減速(2~0m/s, 2m)
  // 1秒後（加速フェーズ終了時点）: 1*1 + 0.5*1*1^2 = 1.5m
  Point predicted =
    crane::getPredictedPositionTrapezoidal(current_pos, current_vel, target_pos, 1.0, 1.0, 4.0);

  EXPECT_DOUBLE_EQ(predicted.x(), 1.5);
  EXPECT_DOUBLE_EQ(predicted.y(), 0.0);
}

TEST(PredictedPositionTrapezoidalTest, Moving_WithCruise)
{
  Point current_pos(0.0, 0.0);
  Vector2 current_vel(1.0, 0.0);
  Point target_pos(7.5, 0.0);

  // 加速度1m/s^2, 最高速度2m/s
  // 1秒加速(1~2m/s, 1.5m) + 2秒等速(2m/s, 4m) + 2秒減速(2~0m/s, 2m)
  // 2秒後（定速フェーズ中、定速1秒目）:
  // 加速: 1.5m + 定速1秒: 2*1 = 2m
  // 合計: 1.5 + 2 = 3.5m
  Point predicted =
    crane::getPredictedPositionTrapezoidal(current_pos, current_vel, target_pos, 2.0, 1.0, 2.0);

  EXPECT_DOUBLE_EQ(predicted.x(), 3.5);
  EXPECT_DOUBLE_EQ(predicted.y(), 0.0);
}

TEST(PredictedPositionTrapezoidalTest, DiagonalMovement)
{
  Point current_pos(0.0, 0.0);
  Vector2 current_vel(0.0, 0.0);
  Point target_pos(3.0, 4.0);  // 距離5m

  // 加速度1m/s^2, 最高速度10m/s（定速区間なし）
  // sqrt(2*1*5) = sqrt(10) ≈ 3.162秒
  // 1秒後（加速フェーズ中）: 0.5 * 1 * 1^2 = 0.5m
  // 方向: (3, 4) / 5 = (0.6, 0.8)
  // 予測位置: (0.5 * 0.6, 0.5 * 0.8) = (0.3, 0.4)
  Point predicted =
    crane::getPredictedPositionTrapezoidal(current_pos, current_vel, target_pos, 1.0, 1.0, 10.0);

  EXPECT_NEAR(predicted.x(), 0.3, 1e-10);
  EXPECT_NEAR(predicted.y(), 0.4, 1e-10);
}

TEST(PredictedPositionTrapezoidalTest, NegativeTime_ReturnsCurrentPosition)
{
  Point current_pos(2.0, 3.0);
  Vector2 current_vel(1.0, 0.0);
  Point target_pos(5.0, 3.0);

  Point predicted =
    crane::getPredictedPositionTrapezoidal(current_pos, current_vel, target_pos, -1.0, 1.0, 2.0);

  EXPECT_DOUBLE_EQ(predicted.x(), 2.0);
  EXPECT_DOUBLE_EQ(predicted.y(), 3.0);
}
// getTravelTimeWithApproachPenalty のテスト
TEST(ApproachPenaltyTest, BallStationary_NoPenalty)
{
  // ボール静止時はペナルティなし（直線計算と同じ）
  Point robot_pos(0.0, 0.0);
  Vector2 robot_vel(0.0, 0.0);
  Point intercept(4.0, 0.0);
  Vector2 ball_vel(0.0, 0.0);

  double penalty_time =
    crane::getTravelTimeWithApproachPenalty(robot_pos, robot_vel, intercept, ball_vel, 1.0, 4.0);
  double straight_time =
    crane::getTravelTimeTrapezoidal(robot_pos, robot_vel, intercept, 1.0, 4.0);

  EXPECT_DOUBLE_EQ(penalty_time, straight_time);
}

TEST(ApproachPenaltyTest, ApproachFromFront_NoPenalty)
{
  // 正面からアプローチ（alignment <= 0）: ペナルティなし
  // ボールは+X方向に進んでいる、ロボットはインターセプト地点の+X側
  Point robot_pos(5.0, 0.0);
  Vector2 robot_vel(0.0, 0.0);
  Point intercept(3.0, 0.0);
  Vector2 ball_vel(1.0, 0.0);  // ボールは+X方向

  // ロボット→インターセプト方向は-X、ボール方向は+X → alignment = -1.0 → ペナルティなし
  double penalty_time =
    crane::getTravelTimeWithApproachPenalty(robot_pos, robot_vel, intercept, ball_vel, 1.0, 4.0);
  double straight_time =
    crane::getTravelTimeTrapezoidal(robot_pos, robot_vel, intercept, 1.0, 4.0);

  EXPECT_DOUBLE_EQ(penalty_time, straight_time);
}

TEST(ApproachPenaltyTest, ApproachFromBehind_HasPenalty)
{
  // 真後ろからアプローチ（alignment ≈ 1.0）: ペナルティあり（直線より長い）
  // ボールは+X方向に進んでいる、ロボットはインターセプト地点の-X側（後方）
  Point robot_pos(-1.0, 0.0);
  Vector2 robot_vel(0.0, 0.0);
  Point intercept(3.0, 0.0);
  Vector2 ball_vel(1.0, 0.0);  // ボールは+X方向

  // ロボット→インターセプト方向は+X、ボール方向も+X → alignment = 1.0 → ペナルティあり
  double penalty_time =
    crane::getTravelTimeWithApproachPenalty(robot_pos, robot_vel, intercept, ball_vel, 1.0, 4.0);
  double straight_time =
    crane::getTravelTimeTrapezoidal(robot_pos, robot_vel, intercept, 1.0, 4.0);

  EXPECT_GT(penalty_time, straight_time);
}

TEST(ApproachPenaltyTest, ApproachFromSide_NoPenalty)
{
  // 横からアプローチ（alignment ≈ 0）: ペナルティなし
  Point robot_pos(3.0, -3.0);
  Vector2 robot_vel(0.0, 0.0);
  Point intercept(3.0, 0.0);
  Vector2 ball_vel(1.0, 0.0);  // ボールは+X方向

  // ロボット→インターセプト方向は+Y、ボール方向は+X → alignment = 0.0 → ペナルティなし
  double penalty_time =
    crane::getTravelTimeWithApproachPenalty(robot_pos, robot_vel, intercept, ball_vel, 1.0, 4.0);
  double straight_time =
    crane::getTravelTimeTrapezoidal(robot_pos, robot_vel, intercept, 1.0, 4.0);

  EXPECT_DOUBLE_EQ(penalty_time, straight_time);
}

}  // namespace crane
