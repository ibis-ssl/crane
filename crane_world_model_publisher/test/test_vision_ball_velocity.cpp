// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <crane_world_model_publisher/calibration/vision_ball_velocity.hpp>
#include <limits>
#include <utility>
#include <vector>

namespace crane::calibration
{
namespace
{
using History = std::vector<std::pair<rclcpp::Time, Ball>>;

auto makeBall(const Point & pos, double pos_z = 0.0) -> Ball
{
  Ball ball{};
  ball.pos = pos;
  ball.pos_z = pos_z;
  ball.vel = Point(0, 0);
  ball.vel_z = 0.0;
  ball.state = Ball::State::STOPPED;
  return ball;
}

// start から step ずつ n 点進む履歴。dt 間隔で並べる
auto makeHistory(const Point & start, const Point & step, size_t n, double dt) -> History
{
  History history;
  for (size_t i = 0; i < n; ++i) {
    history.emplace_back(
      rclcpp::Time(static_cast<int64_t>(i * dt * 1e9)),
      makeBall(start + step * static_cast<double>(i)));
  }
  return history;
}

TEST(VisionBallVelocity, ConstantVelocityKeepsSignAndMagnitude)
{
  constexpr double dt = 0.01;
  // 片側だけ平滑化すると過去 4 点が効くので、履歴を 5 点以上にして検出できるようにする
  for (const Point & velocity : {Point(1.0, 0.0), Point(-2.0, 0.5), Point(0.0, -3.0)}) {
    const auto history = makeHistory(Point(0.3, -0.2), velocity * dt, 6, dt);
    const Ball current = makeBall(history.back().second.pos + velocity * dt);

    const auto result = computeVisionVelocity(history, current, dt);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->vel.x(), velocity.x(), 1e-9);
    EXPECT_NEAR(result->vel.y(), velocity.y(), 1e-9);
    EXPECT_NEAR(result->vel_z, 0.0, 1e-9);
  }
}

TEST(VisionBallVelocity, StationaryNoiseIsDifferencedWithoutScaling)
{
  constexpr double dt = 0.016;
  // 静止ボールに ±0.2mm のノイズ。差分は 0.4mm / 16ms = 0.025m/s で、STOPPED の閾値 0.05m/s 未満
  History history;
  for (int i = 0; i < 6; ++i) {
    const double noise = (i % 2 == 0) ? 0.0002 : -0.0002;
    history.emplace_back(
      rclcpp::Time(static_cast<int64_t>(i * dt * 1e9)), makeBall(Point(1.0 + noise, 2.0)));
  }
  Ball current = makeBall(Point(1.0 + 0.0002, 2.0));  // 末尾（i=5）は -0.2mm

  const auto result = computeVisionVelocity(history, current, dt);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->vel.x(), 0.0004 / dt, 1e-9);
  EXPECT_NEAR(result->vel.y(), 0.0, 1e-9);

  current.vel = result->vel;
  current.vel_z = result->vel_z;
  EXPECT_EQ(classifyBallState(current), Ball::State::STOPPED);
}

TEST(VisionBallVelocity, DtOutsideRangeIsRejected)
{
  const auto history = makeHistory(Point(0, 0), Point(0.01, 0), 3, 0.01);
  const Ball current = makeBall(Point(0.03, 0));

  // 範囲の両端は含む
  EXPECT_TRUE(computeVisionVelocity(history, current, VISION_VELOCITY_MIN_DT).has_value());
  EXPECT_TRUE(computeVisionVelocity(history, current, VISION_VELOCITY_MAX_DT).has_value());

  for (const double dt :
       {0.0, -0.01, 5e-6, 0.2001, 1.0, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(computeVisionVelocity(history, current, dt).has_value()) << "dt=" << dt;
  }
  EXPECT_FALSE(computeVisionVelocity(History{}, current, 0.01).has_value());
}

TEST(VisionBallVelocity, InvalidDtStopsAfterGapAndHoldsOtherwise)
{
  Ball prev = makeBall(Point(0, 0), 0.05);
  prev.vel = Point(2.0, -1.0);
  prev.vel_z = 0.3;
  prev.state = Ball::State::FLYING;

  Ball after_gap = makeBall(Point(1, 1));
  holdVelocityForInvalidDt(prev, 0.25, after_gap);
  EXPECT_DOUBLE_EQ(after_gap.vel.norm(), 0.0);
  EXPECT_DOUBLE_EQ(after_gap.vel_z, 0.0);
  EXPECT_EQ(after_gap.state, Ball::State::STOPPED);

  for (const double dt : {5e-6, 0.0, -0.01, std::numeric_limits<double>::quiet_NaN()}) {
    Ball too_short = makeBall(Point(1, 1));
    holdVelocityForInvalidDt(prev, dt, too_short);
    EXPECT_DOUBLE_EQ(too_short.vel.x(), 2.0) << "dt=" << dt;
    EXPECT_DOUBLE_EQ(too_short.vel.y(), -1.0) << "dt=" << dt;
    EXPECT_DOUBLE_EQ(too_short.vel_z, 0.3) << "dt=" << dt;
    EXPECT_EQ(too_short.state, Ball::State::FLYING) << "dt=" << dt;
  }
}

TEST(VisionBallVelocity, ZDifferenceAndStateClassification)
{
  constexpr double dt = 0.01;
  // z が 0.5m/s で上昇している（x は 1m/s）
  History history;
  for (int i = 0; i < 6; ++i) {
    history.emplace_back(
      rclcpp::Time(static_cast<int64_t>(i * dt * 1e9)), makeBall(Point(0.01 * i, 0), 0.005 * i));
  }
  Ball current = makeBall(Point(0.06, 0), 0.03);
  const auto result = computeVisionVelocity(history, current, dt);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->vel_z, 0.5, 1e-9);
  current.vel = result->vel;
  current.vel_z = result->vel_z;
  EXPECT_EQ(classifyBallState(current), Ball::State::FLYING);

  auto classify = [](double speed, double pos_z, double vel_z) {
    Ball ball = makeBall(Point(0, 0), pos_z);
    ball.vel = Point(speed, 0);
    ball.vel_z = vel_z;
    return classifyBallState(ball);
  };
  // 平面速度が 0.05m/s 未満なら z に関係なく STOPPED
  EXPECT_EQ(classify(0.049, 0.1, 1.0), Ball::State::STOPPED);
  EXPECT_EQ(classify(1.0, 0.0, 0.0), Ball::State::ROLLING);
  // FLYING の閾値（pos_z > 0.02, |vel_z| > 0.1）はどちらも境界を含まない
  EXPECT_EQ(classify(1.0, 0.02, 0.1), Ball::State::ROLLING);
  EXPECT_EQ(classify(1.0, 0.021, 0.0), Ball::State::FLYING);
  EXPECT_EQ(classify(1.0, 0.0, 0.11), Ball::State::FLYING);
  EXPECT_EQ(classify(1.0, 0.0, -0.11), Ball::State::FLYING);
}
}  // namespace
}  // namespace crane::calibration
