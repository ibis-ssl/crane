// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_basics/ball_info.hpp>
#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/interval.hpp>
#include <crane_basics/pid_controller.hpp>
#include <crane_basics/robot_info.hpp>
#include <crane_basics/time.hpp>
#include <crane_basics/travel_time.hpp>

namespace crane
{
// Circleのテスト
TEST(CircleTest, CreateAndMeasure)
{
  crane::Circle circle;
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

// PIDControllerのテスト
TEST(PIDControllerTest, BasicControl)
{
  PIDController pid;
  pid.setGain(1.0, 0.5, 0.1);  // P=1.0, I=0.5, D=0.1

  // 初期エラー
  double out = pid.update(2.0, 0.1);
  // P項: 1.0 * 2.0 = 2.0
  // I項: 0.5 * 2.0 * 0.1 = 0.1
  // D項: 0.1 * (2.0 - 0.0) / 0.1 = 2.0
  EXPECT_DOUBLE_EQ(out, 4.1);

  // 2回目の更新（エラー減少）
  out = pid.update(1.0, 0.1);
  // P項: 1.0 * 1.0 = 1.0
  // I項: 0.5 * (2.0*0.1 + 1.0*0.1) = 0.15
  // D項: 0.1 * (1.0 - 2.0) / 0.1 = -1.0
  EXPECT_DOUBLE_EQ(out, 0.15);

  // 3回目の更新（エラー増加）
  out = pid.update(1.5, 0.1);
  // P項: 1.0 * 1.5 = 1.5
  // I項: 0.5 * (2.0*0.1 + 1.0*0.1 + 1.5*0.1) = 0.225
  // D項: 0.1 * (1.5 - 1.0) / 0.1 = 0.5
  EXPECT_DOUBLE_EQ(out, 2.225);
}

TEST(PIDControllerTest, IntegralClamp)
{
  PIDController pid;
  pid.setGain(1.0, 1.0, 0.0, 0.5);  // max_integral = 0.5

  // 積分項が制限を超えないケース
  double out = pid.update(0.2, 0.1);
  // P項: 1.0 * 0.2 = 0.2
  // I項: 1.0 * 0.2 * 0.1 = 0.02
  // D項: 0.0
  EXPECT_DOUBLE_EQ(out, 0.22);

  // 積分項が制限に達するケース
  out = pid.update(5.0, 0.1);
  // P項: 1.0 * 5.0 = 5.0
  // I項（制限前）: 1.0 * (0.2*0.1 + 5.0*0.1) = 0.52
  // I項（制限後）: 0.5
  // D項: 0.0
  EXPECT_DOUBLE_EQ(out, 5.5);

  // 積分項がすでに最大値なので変わらない
  out = pid.update(5.0, 0.1);
  // P項: 1.0 * 5.0 = 5.0
  // I項: 0.5 (クランプ済み)
  // D項: 0.0
  EXPECT_DOUBLE_EQ(out, 5.5);
}

// Intervalクラスのテスト
TEST(IntervalTest, AppendIntervals)
{
  Interval interval;

  // 単一の区間を追加
  interval.append(1.0, 3.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 2.0);

  // 重複しない区間を追加
  interval.append(5.0, 7.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 4.0);

  // 重複する区間を追加
  interval.append(2.0, 6.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 6.0);

  // 最大区間の確認
  auto largest = interval.getLargestInterval();
  EXPECT_DOUBLE_EQ(largest.first, 1.0);
  EXPECT_DOUBLE_EQ(largest.second, 7.0);
}

TEST(IntervalTest, EraseIntervals)
{
  Interval interval;

  // 初期区間を設定
  interval.append(0.0, 10.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 10.0);

  // 内部区間を削除
  interval.erase(3.0, 5.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 8.0);

  // 端の区間を削除
  interval.erase(-1.0, 2.0);
  EXPECT_DOUBLE_EQ(interval.getWidth(), 6.0);

  // 最大区間の確認
  auto largest = interval.getLargestInterval();
  EXPECT_DOUBLE_EQ(largest.first, 5.0);
  EXPECT_DOUBLE_EQ(largest.second, 10.0);
}

// geometry_operationsのテスト追加
TEST(GeometryOperationsTest, NormalizeAngle)
{
  // 正の角度の正規化
  EXPECT_NEAR(normalizeAngle(3.5 * M_PI), -0.5 * M_PI, 1e-10);

  // 負の角度の正規化
  EXPECT_NEAR(normalizeAngle(-3.5 * M_PI), 0.5 * M_PI, 1e-10);

  // -π〜πの範囲内の角度は変わらない
  EXPECT_DOUBLE_EQ(normalizeAngle(0.5), 0.5);
  EXPECT_DOUBLE_EQ(normalizeAngle(-0.5), -0.5);
}

TEST(GeometryOperationsTest, GetAngleDiff)
{
  // 単純な差
  EXPECT_DOUBLE_EQ(getAngleDiff(0.5, 0.3), 0.2);

  // -πとπの間の差（境界を超える）
  EXPECT_NEAR(getAngleDiff(M_PI - 0.1, -M_PI + 0.1), -0.2, 1e-10);

  // Pose2D間の角度差
  Pose2D pose1, pose2;
  pose1.theta = 0.5;
  pose2.theta = -0.5;
  EXPECT_DOUBLE_EQ(getAngleDiff(pose1, pose2), 1.0);

  // Pose2DとDouble間の角度差
  EXPECT_DOUBLE_EQ(getAngleDiff(pose1, 0.0), 0.5);
  EXPECT_DOUBLE_EQ(getAngleDiff(0.0, pose1), -0.5);
}

TEST(GeometryOperationsTest, GetIntermediateAngle)
{
  // 単純な中間角度
  EXPECT_DOUBLE_EQ(getIntermediateAngle(0.0, 1.0), 0.5);

  // -πとπの間の中間角度（境界を超える）
  EXPECT_NEAR(getIntermediateAngle(M_PI - 0.1, -M_PI + 0.1), M_PI, 1e-10);
}

TEST(GeometryOperationsTest, GetCircle)
{
  // 3点から円を作成
  // (1,0)を中心とする半径1の円
  Point p1(0, 0);
  Point p2(2, 0);
  Point p3(1, 1);
  auto circle = getCircle(p1, p2, p3);

  ASSERT_TRUE(circle.has_value());
  EXPECT_NEAR(circle->center.x(), 1.0, 1e-10);
  EXPECT_NEAR(circle->center.y(), 0.0, 1e-10);
  EXPECT_NEAR(circle->radius, 1.0, 1e-10);

  // 一直線上の3点からは円を作成できない
  Point p4(3, 0);
  auto invalid_circle = getCircle(p1, p2, p4);
  EXPECT_FALSE(invalid_circle.has_value());
}

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
  Point test_point(5.0, 3.0);
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

// Ball構造体のテスト
TEST(BallTest, MovementTests)
{
  Ball ball;
  ball.pos << 0.0, 0.0;
  ball.vel << 0.0, 0.0;
  ball.detected = true;

  // 停止状態のテスト
  EXPECT_FALSE(ball.isMoving());
  EXPECT_TRUE(ball.isStopped());

  // 移動中のテスト
  ball.vel << 1.0, 0.0;
  EXPECT_TRUE(ball.isMoving());
  EXPECT_FALSE(ball.isStopped());

  // カスタム閾値でのテスト
  ball.vel << 0.005, 0.0;
  EXPECT_FALSE(ball.isMoving(0.01));  // 0.01より小さいので停止とみなす
  EXPECT_TRUE(ball.isMoving(0.001));  // 0.001より大きいので移動中とみなす
}

TEST(BallTest, DirectionTests)
{
  Ball ball;
  ball.pos << 0.0, 0.0;
  ball.detected = true;

  // X方向に動いている場合
  ball.vel << 1.0, 0.0;

  // X正方向のポイントに向かっている
  Point target_point1(5.0, 0.0);
  EXPECT_TRUE(ball.isMovingTowards(target_point1));
  EXPECT_FALSE(ball.isMovingAwayFrom(target_point1));

  // X負方向のポイントから離れている
  Point target_point2(-5.0, 0.0);
  EXPECT_FALSE(ball.isMovingTowards(target_point2));
  EXPECT_TRUE(ball.isMovingAwayFrom(target_point2));

  // Y方向のポイント - 角度が60度以上なので向かっていない
  Point target_point3(0.0, 5.0);
  EXPECT_FALSE(ball.isMovingTowards(target_point3));
  EXPECT_FALSE(ball.isMovingAwayFrom(target_point3));

  // 45度方向のポイント - 角度が60度未満なので向かっている
  Point target_point4(5.0, 5.0);
  EXPECT_TRUE(ball.isMovingTowards(target_point4));
  EXPECT_FALSE(ball.isMovingAwayFrom(target_point4));

  // 閾値を30度に設定して再テスト
  EXPECT_FALSE(ball.isMovingTowards(target_point4, 30.0));
}

// Hysteresisクラスのテスト
TEST(HysteresisTest, StateTransition)
{
  Hysteresis hysteresis(0.3, 0.7);

  // 初期状態はLow
  EXPECT_FALSE(hysteresis.is_high);

  // 中間値 (0.3 < 0.5 < 0.7) を与えても状態は変わらない
  hysteresis.update(0.5);
  EXPECT_FALSE(hysteresis.is_high);

  // 上限値を超えるとHighになる
  hysteresis.update(0.8);
  EXPECT_TRUE(hysteresis.is_high);

  // 中間値 (0.3 < 0.5 < 0.7) を与えても状態は変わらない
  hysteresis.update(0.5);
  EXPECT_TRUE(hysteresis.is_high);

  // 下限値を下回るとLowになる
  hysteresis.update(0.2);
  EXPECT_FALSE(hysteresis.is_high);
}

// コールバックのテスト
TEST(HysteresisTest, Callbacks)
{
  Hysteresis hysteresis(0.3, 0.7);
  bool upper_called = false;
  bool lower_called = false;

  hysteresis.upper_callback = [&upper_called]() { upper_called = true; };
  hysteresis.lower_callback = [&lower_called]() { lower_called = true; };

  // 上限値を超えるとupper_callbackが呼ばれる
  hysteresis.update(0.8);
  EXPECT_TRUE(upper_called);
  EXPECT_FALSE(lower_called);

  // リセット
  upper_called = false;
  lower_called = false;

  // 上限値を超えてもすでにHighなので何も呼ばれない
  hysteresis.update(0.9);
  EXPECT_FALSE(upper_called);
  EXPECT_FALSE(lower_called);

  // 下限値を下回るとlower_callbackが呼ばれる
  hysteresis.update(0.2);
  EXPECT_FALSE(upper_called);
  EXPECT_TRUE(lower_called);
}

// time.hppの関数テスト
TEST(TimeTest, GetDiffSec)
{
  auto now = std::chrono::high_resolution_clock::now();
  auto later = now + std::chrono::milliseconds(100);

  // 差分は約0.1秒
  double diff = getDiffSec(now, later);
  EXPECT_NEAR(diff, 0.1, 0.01);

  // 順序が逆でも絶対値なので同じ
  double diff2 = getDiffSec(later, now);
  EXPECT_NEAR(diff2, 0.1, 0.01);
}

// getElapsedSecのテストはtickに依存するため省略
// ScopedTimerのテストも外部依存が多いため省略

// テンプレート関数のインスタンス化テスト
TEST(TimeTest, TemplateInstantiation)
{
  using TestClock = std::chrono::steady_clock;

  auto start = TestClock::now();
  auto end = start + std::chrono::seconds(1);

  double diff = getDiffSec<TestClock>(start, end);
  EXPECT_DOUBLE_EQ(diff, 1.0);
}
}  // namespace crane

// メイン関数
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
