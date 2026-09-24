// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <crane_physics/ball_info.hpp>
#include <crane_physics/ball_physics_model.hpp>
#include <utility>

namespace crane
{
// Ball::ParabolicPhysicsクラスのテスト
class BallParabolicPhysicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // テスト用のセットアップ
  }

  BallParabolicPhysicsTest() = default;

  // テストヘルパー関数
  Ball createBall(const Point3D & pos, const Point3D & vel)
  {
    Ball ball;
    ball.pos = Point(pos.x(), pos.y());
    ball.vel = Point(vel.x(), vel.y());
    ball.pos_z = pos.z();
    ball.vel_z = vel.z();
    ball.state = Ball::State::FLYING;
    return ball;
  }
};

TEST_F(BallParabolicPhysicsTest, EstimateInitialPointsFromTwoPoints)
{
  // 2点からの初期速度推定テスト
  auto ball = createBall(Point3D(0.0, 0.0, 0.0), Point3D(0.0, 0.0, 0.0));
  auto physics = ball.getParabolicPhysics();
  physics.point_log.push_back({Point3D(0.0, 0.0, 1.0), 0.0});
  physics.point_log.push_back({Point3D(1.0, 0.5, 1.5), 0.1});  // 0.1秒後の理論位置

  physics.estimateInitialVelocityFromPointLog();

  auto estimated_pos_1 = physics.getPredictedPosition3D(0.0);
  auto estimated_pos_2 = physics.getPredictedPosition3D(0.1);

  // 期待値: 設定した点を通る解が正しく計算されているかを確認
  EXPECT_NEAR(estimated_pos_1.x(), 0.0, 0.01);
  EXPECT_NEAR(estimated_pos_1.y(), 0.0, 0.01);
  EXPECT_NEAR(estimated_pos_1.z(), 1.0, 0.01);
  EXPECT_NEAR(estimated_pos_2.x(), 1.0, 0.01);
  EXPECT_NEAR(estimated_pos_2.y(), 0.5, 0.01);
  EXPECT_NEAR(estimated_pos_2.z(), 1.5, 0.01);
}

TEST_F(BallParabolicPhysicsTest, EstimateInitialVelocityFromMultiplePoints)
{
  // 複数点からの最小二乗法による初期速度推定テスト
  constexpr double gravity = -9.81;
  Point3D init_pos(0.0, 0.0, 2.0);
  Point3D init_vel(8.0, 4.0, 6.0);

  auto ball = createBall(Point3D(0.0, 0.0, 0.0), Point3D(0.0, 0.0, 0.0));
  auto physics = ball.getParabolicPhysics();

  // 理論的な放物運動軌道を生成
  std::vector<double> times = {0.0, 0.1, 0.2, 0.3, 0.4};
  for (double t : times) {
    Point3D pos;
    pos.x() = init_pos.x() + init_vel.x() * t;
    pos.y() = init_pos.y() + init_vel.y() * t;
    pos.z() = init_pos.z() + init_vel.z() * t + 0.5 * gravity * t * t;
    physics.point_log.push_back({pos, t});
  }

  physics.estimateInitialVelocityFromPointLog();

  // 推定後の軌道が元の軌道と一致するかを確認
  auto pos_at_02 = physics.getPredictedPosition3D(0.2);
  Point3D expected_pos_at_02;
  expected_pos_at_02.x() = init_pos.x() + init_vel.x() * 0.2;
  expected_pos_at_02.y() = init_pos.y() + init_vel.y() * 0.2;
  expected_pos_at_02.z() = init_pos.z() + init_vel.z() * 0.2 + 0.5 * gravity * 0.2 * 0.2;

  EXPECT_NEAR(pos_at_02.x(), expected_pos_at_02.x(), 0.1);
  EXPECT_NEAR(pos_at_02.y(), expected_pos_at_02.y(), 0.1);
  EXPECT_NEAR(pos_at_02.z(), expected_pos_at_02.z(), 0.1);
}

TEST_F(BallParabolicPhysicsTest, GetGroundPointNormalTrajectory)
{
  // 通常の放物軌道での着地点計算テスト
  auto ball = createBall(Point3D(0.0, 0.0, 5.0), Point3D(10.0, 5.0, 8.0));
  auto physics = ball.getParabolicPhysics();

  auto ground_point = physics.getGroundPoint();

  // 理論的な着地時間: z(t) = 5 + 8*t - 4.905*t^2 = 0
  // 4.905*t^2 - 8*t - 5 = 0
  // t = (8 + sqrt(64 + 98.1)) / 9.81 ≈ 1.494秒
  double expected_time = (8.0 + std::sqrt(64.0 + 4 * 4.905 * 5.0)) / 9.81;

  EXPECT_NEAR(ground_point.time, expected_time, 0.01);

  // 着地位置
  EXPECT_NEAR(ground_point.position.x(), 10.0 * expected_time, 0.1);
  EXPECT_NEAR(ground_point.position.y(), 5.0 * expected_time, 0.1);
  EXPECT_DOUBLE_EQ(ground_point.position.z(), 0.0);
}

TEST_F(BallParabolicPhysicsTest, GetGroundPointAlreadyOnGround)
{
  // すでに地面にいる場合のテスト
  auto ball = createBall(Point3D(2.0, 3.0, 0.0), Point3D(1.0, 1.0, 1.0));
  auto physics = ball.getParabolicPhysics();

  auto ground_point = physics.getGroundPoint();

  EXPECT_DOUBLE_EQ(ground_point.time, 0.0);
  EXPECT_DOUBLE_EQ(ground_point.position.x(), 2.0);
  EXPECT_DOUBLE_EQ(ground_point.position.y(), 3.0);
  EXPECT_DOUBLE_EQ(ground_point.position.z(), 0.0);
}

TEST_F(BallParabolicPhysicsTest, GetGroundPointDownwardTrajectory)
{
  // 下向きの軌道での着地点計算テスト
  auto ball = createBall(Point3D(0.0, 0.0, 3.0), Point3D(5.0, 2.0, -2.0));
  auto physics = ball.getParabolicPhysics();

  auto ground_point = physics.getGroundPoint();

  // 理論的な着地時間: z(t) = 3 - 2*t - 4.905*t^2 = 0
  // 4.905*t^2 + 2*t - 3 = 0
  // t = (-2 + sqrt(4 + 58.86)) / 9.81
  double expected_time = (-2.0 + std::sqrt(4.0 + 4 * 4.905 * 3.0)) / 9.81;

  EXPECT_NEAR(ground_point.time, expected_time, 0.01);
  EXPECT_NEAR(ground_point.position.x(), 5.0 * expected_time, 0.1);
  EXPECT_NEAR(ground_point.position.y(), 2.0 * expected_time, 0.1);
  EXPECT_DOUBLE_EQ(ground_point.position.z(), 0.0);
}

TEST_F(BallParabolicPhysicsTest, GetGroundPointNoGroundImpact)
{
  // 地面に到達しない軌道（最高点を返す）
  auto ball = createBall(Point3D(0.0, 0.0, -1.0), Point3D(5.0, 2.0, 2.0));
  auto physics = ball.getParabolicPhysics();

  auto ground_point = physics.getGroundPoint();

  // 最高点での時間: t = -vz/g = -2.0/(-9.81) ≈ 0.204秒
  double expected_peak_time = 2.0 / 9.81;

  EXPECT_NEAR(ground_point.time, expected_peak_time, 0.01);

  // 最高点での位置
  EXPECT_NEAR(ground_point.position.x(), 5.0 * expected_peak_time, 0.1);
  EXPECT_NEAR(ground_point.position.y(), 2.0 * expected_peak_time, 0.1);

  // Z座標は初期位置 + 初期速度*時間 + 0.5*重力*時間^2
  double expected_z =
    -1.0 + 2.0 * expected_peak_time - 0.5 * 9.81 * expected_peak_time * expected_peak_time;
  EXPECT_NEAR(ground_point.position.z(), expected_z, 0.1);
}

TEST_F(BallParabolicPhysicsTest, GroundPointRisingFromBelowGroundUsesDescendingRoot)
{
  // 根は約 0.005 秒（上昇中の地面通過）と約 0.403 秒（下降して着地）。着地は後者
  auto ball = createBall(Point3D(0.0, 0.0, -0.01), Point3D(1.0, 0.0, 2.0));
  auto physics = ball.getParabolicPhysics();
  const double expected_time = (2.0 + std::sqrt(4.0 - 2.0 * 9.81 * 0.01)) / 9.81;

  auto [landing_pos, landing_time] = physics.getGroundIntersection();
  EXPECT_NEAR(landing_time, expected_time, 1e-9);
  EXPECT_NEAR(landing_pos.x(), expected_time, 1e-9);

  auto ground_point = physics.getGroundPoint();
  EXPECT_NEAR(ground_point.time, expected_time, 1e-9);
  EXPECT_NEAR(ground_point.position.x(), expected_time, 1e-9);
}

TEST_F(BallParabolicPhysicsTest, EstimateInitialVelocityInsufficientData)
{
  // データが不十分な場合のテスト
  auto ball = createBall(Point3D(0.0, 0.0, 0.0), Point3D(0.0, 0.0, 0.0));
  auto physics = ball.getParabolicPhysics();
  physics.point_log.push_back({Point3D(0.0, 0.0, 1.0), 0.0});

  physics.estimateInitialVelocityFromPointLog();

  // データが不十分な場合の動作を確認（位置計算で代用）
  auto pos_at_0 = physics.getPredictedPosition3D(0.0);
  EXPECT_DOUBLE_EQ(pos_at_0.x(), 0.0);
  EXPECT_DOUBLE_EQ(pos_at_0.y(), 0.0);
  EXPECT_DOUBLE_EQ(pos_at_0.z(), 1.0);  // 初期位置が更新されている
}

TEST_F(BallParabolicPhysicsTest, EstimateInitialVelocityNumericalStability)
{
  // 数値的に不安定なケースのテスト（すべて同じ時間）
  auto ball = createBall(Point3D(0.0, 0.0, 0.0), Point3D(0.0, 0.0, 0.0));
  auto physics = ball.getParabolicPhysics();
  physics.point_log.push_back({Point3D(0.0, 0.0, 1.0), 0.0});
  physics.point_log.push_back({Point3D(1.0, 1.0, 1.0), 0.0});
  physics.point_log.push_back({Point3D(2.0, 2.0, 1.0), 0.0});

  physics.estimateInitialVelocityFromPointLog();

  // 数値的に不安定な場合の動作を確認
  auto pos_at_0 = physics.getPredictedPosition3D(0.0);
  EXPECT_DOUBLE_EQ(pos_at_0.x(), 0.0);
  EXPECT_DOUBLE_EQ(pos_at_0.y(), 0.0);
  EXPECT_DOUBLE_EQ(pos_at_0.z(), 1.0);
}

// BallPhysicsModel の FLYING 着地時刻。
// xy 速度 0 なら転がりが無く、getStopTime が着地時刻そのものになる
class BallPhysicsModelFlyingLandingTest : public ::testing::Test
{
protected:
  BallPhysicsModel model_ = BallPhysicsModel::createDefault();  // gravity = -9.81

  [[nodiscard]] auto landingTime(double pos_z, double vel_z) const -> double
  {
    return model_.getStopTime(Point(0.0, 0.0), Ball::State::FLYING, pos_z, vel_z);
  }
};

TEST_F(BallPhysicsModelFlyingLandingTest, DescendingFromHeight)
{
  // z(t) = 1 - t - 4.905 t^2 = 0 の正の根。pos_z を無視する旧式では 0 になっていた
  EXPECT_NEAR(landingTime(1.0, -1.0), (-1.0 + std::sqrt(1.0 + 2.0 * 9.81 * 1.0)) / 9.81, 1e-9);
}

TEST_F(BallPhysicsModelFlyingLandingTest, RisingFromGround)
{
  EXPECT_NEAR(landingTime(0.0, 2.0), 2.0 * 2.0 / 9.81, 1e-9);
}

TEST_F(BallPhysicsModelFlyingLandingTest, RisingFromBelowGroundUsesDescendingRoot)
{
  // 根は約 0.005 秒（上昇中の地面通過）と約 0.403 秒（下降して着地）。着地は後者
  EXPECT_NEAR(landingTime(-0.01, 2.0), (2.0 + std::sqrt(4.0 - 2.0 * 9.81 * 0.01)) / 9.81, 1e-9);
}

TEST_F(BallPhysicsModelFlyingLandingTest, NeverReachesGround)
{
  // 判別式が負: pos_z < 0 のまま地面まで上がらない
  EXPECT_EQ(landingTime(-1.0, 1.0), 0.0);
}

TEST_F(BallPhysicsModelFlyingLandingTest, PredictionsSwitchToRollingAtLandingTime)
{
  const Point position(0.0, 0.0);
  const Point velocity(1.0, 0.0);
  for (const auto [pos_z, vel_z] : {std::pair{1.0, -1.0}, std::pair{-0.01, 2.0}}) {
    const double landing_time = landingTime(pos_z, vel_z);
    const auto state = Ball::State::FLYING;

    // 着地までは xy 等速
    EXPECT_NEAR(
      model_.predictPosition(position, velocity, state, pos_z, vel_z, landing_time).x(),
      landing_time, 1e-9);
    EXPECT_EQ(
      model_.predictVelocity(position, velocity, state, pos_z, vel_z, landing_time - 1e-3),
      velocity);
    // 着地後は転がりで減速する（deceleration = 0.36）
    EXPECT_NEAR(
      model_.predictVelocity(position, velocity, state, pos_z, vel_z, landing_time + 0.1).x(),
      1.0 - 0.36 * 0.1, 1e-9);
    // 停止時刻の予測位置が最大到達距離と一致する
    const double stop_time = model_.getStopTime(velocity, state, pos_z, vel_z);
    EXPECT_NEAR(
      model_.predictPosition(position, velocity, state, pos_z, vel_z, stop_time).x(),
      model_.getMaxDistance(position, velocity, state, pos_z, vel_z), 1e-9);
  }
}
}  // namespace crane
