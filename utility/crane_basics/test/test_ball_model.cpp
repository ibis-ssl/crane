// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <crane_basics/ball_model.hpp>

namespace crane
{
// ParabolicBallPhysicsクラスのテスト
class ParabolicBallPhysicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // テスト用のセットアップ
  }

  ParabolicBallPhysicsTest() = default;

  // テストヘルパークラス（ParabolicBallPhysicsクラスの protected メンバーにアクセスするため）
  class TestableParabolicBallPhysics : public ParabolicBallPhysics
  {
  public:
    TestableParabolicBallPhysics() : ParabolicBallPhysics(Point3D(0, 0, 0), Point3D(0, 0, 0)) {}

    void setInitialConditions(const Point3D & pos, const Point3D & vel)
    {
      initial_position_ = pos;
      initial_velocity_ = vel;
    }

    void addPoint(const Point3D & pos, double time) { point_log.push_back({pos, time}); }

    void callEstimateInitialVelocity() { estimateInitialVelocityFromPointLog(); }

    Point3DStamped callGetGroundPoint() { return getGroundPoint(); }

    Point3D getInitialPosition() const { return initial_position_; }
    Point3D getInitialVelocity() const { return initial_velocity_; }
  };

  TestableParabolicBallPhysics physics;
};

TEST_F(ParabolicBallPhysicsTest, EstimateInitialVelocityFromTwoPoints)
{
  // 2点からの初期速度推定テスト
  physics.addPoint(Point3D(0.0, 0.0, 1.0), 0.0);
  physics.addPoint(Point3D(1.0, 0.5, 1.4095), 0.1);  // 0.1秒後の理論位置

  physics.callEstimateInitialVelocity();

  auto estimated_vel = physics.getInitialVelocity();

  // 期待値: 重力補正を考慮した初期速度
  // v_x = dx/dt = 1.0/0.1 = 10.0
  // v_y = dy/dt = 0.5/0.1 = 5.0
  // v_z = dz/dt + 0.5*g*dt = (1.4095-1.0)/0.1 + 0.5*9.81*0.1 = 4.095 + 0.4905 ≈ 4.586
  EXPECT_NEAR(estimated_vel.x(), 10.0, 0.01);
  EXPECT_NEAR(estimated_vel.y(), 5.0, 0.01);
  EXPECT_NEAR(estimated_vel.z(), 4.586, 0.1);
}

TEST_F(ParabolicBallPhysicsTest, EstimateInitialVelocityFromMultiplePoints)
{
  // 複数点からの最小二乗法による初期速度推定テスト
  constexpr double gravity = -9.81;
  Point3D init_pos(0.0, 0.0, 2.0);
  Point3D init_vel(8.0, 4.0, 6.0);

  // 理論的な放物運動軌道を生成
  std::vector<double> times = {0.0, 0.1, 0.2, 0.3, 0.4};
  for (double t : times) {
    Point3D pos;
    pos.x() = init_pos.x() + init_vel.x() * t;
    pos.y() = init_pos.y() + init_vel.y() * t;
    pos.z() = init_pos.z() + init_vel.z() * t + 0.5 * gravity * t * t;
    physics.addPoint(pos, t);
  }

  physics.callEstimateInitialVelocity();

  auto estimated_pos = physics.getInitialPosition();
  auto estimated_vel = physics.getInitialVelocity();

  // 推定された初期位置と速度が理論値に近いことを確認
  EXPECT_NEAR(estimated_pos.x(), init_pos.x(), 0.01);
  EXPECT_NEAR(estimated_pos.y(), init_pos.y(), 0.01);
  EXPECT_NEAR(estimated_pos.z(), init_pos.z(), 0.01);

  EXPECT_NEAR(estimated_vel.x(), init_vel.x(), 0.1);
  EXPECT_NEAR(estimated_vel.y(), init_vel.y(), 0.1);
  EXPECT_NEAR(estimated_vel.z(), init_vel.z(), 0.1);
}

TEST_F(ParabolicBallPhysicsTest, GetGroundPointNormalTrajectory)
{
  // 通常の放物軌道での着地点計算テスト
  physics.setInitialConditions(Point3D(0.0, 0.0, 5.0), Point3D(10.0, 5.0, 8.0));

  auto ground_point = physics.callGetGroundPoint();

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

TEST_F(ParabolicBallPhysicsTest, GetGroundPointAlreadyOnGround)
{
  // すでに地面にいる場合のテスト
  physics.setInitialConditions(Point3D(2.0, 3.0, 0.0), Point3D(1.0, 1.0, 1.0));

  auto ground_point = physics.callGetGroundPoint();

  EXPECT_DOUBLE_EQ(ground_point.time, 0.0);
  EXPECT_DOUBLE_EQ(ground_point.position.x(), 2.0);
  EXPECT_DOUBLE_EQ(ground_point.position.y(), 3.0);
  EXPECT_DOUBLE_EQ(ground_point.position.z(), 0.0);
}

TEST_F(ParabolicBallPhysicsTest, GetGroundPointDownwardTrajectory)
{
  // 下向きの軌道での着地点計算テスト
  physics.setInitialConditions(Point3D(0.0, 0.0, 3.0), Point3D(5.0, 2.0, -2.0));

  auto ground_point = physics.callGetGroundPoint();

  // 理論的な着地時間: z(t) = 3 - 2*t - 4.905*t^2 = 0
  // 4.905*t^2 + 2*t - 3 = 0
  // t = (-2 + sqrt(4 + 58.86)) / 9.81
  double expected_time = (-2.0 + std::sqrt(4.0 + 4 * 4.905 * 3.0)) / 9.81;

  EXPECT_NEAR(ground_point.time, expected_time, 0.01);
  EXPECT_NEAR(ground_point.position.x(), 5.0 * expected_time, 0.1);
  EXPECT_NEAR(ground_point.position.y(), 2.0 * expected_time, 0.1);
  EXPECT_DOUBLE_EQ(ground_point.position.z(), 0.0);
}

TEST_F(ParabolicBallPhysicsTest, GetGroundPointNoGroundImpact)
{
  // 地面に到達しない軌道（最高点を返す）
  physics.setInitialConditions(Point3D(0.0, 0.0, -1.0), Point3D(5.0, 2.0, 2.0));

  auto ground_point = physics.callGetGroundPoint();

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

TEST_F(ParabolicBallPhysicsTest, EstimateInitialVelocityInsufficientData)
{
  // データが不十分な場合のテスト
  physics.addPoint(Point3D(0.0, 0.0, 1.0), 0.0);

  physics.callEstimateInitialVelocity();

  auto estimated_vel = physics.getInitialVelocity();

  // データが不十分な場合はゼロベクトルが設定される
  EXPECT_DOUBLE_EQ(estimated_vel.x(), 0.0);
  EXPECT_DOUBLE_EQ(estimated_vel.y(), 0.0);
  EXPECT_DOUBLE_EQ(estimated_vel.z(), 0.0);
}

TEST_F(ParabolicBallPhysicsTest, EstimateInitialVelocityNumericalStability)
{
  // 数値的に不安定なケースのテスト（すべて同じ時間）
  physics.addPoint(Point3D(0.0, 0.0, 1.0), 0.0);
  physics.addPoint(Point3D(1.0, 1.0, 1.0), 0.0);
  physics.addPoint(Point3D(2.0, 2.0, 1.0), 0.0);

  physics.callEstimateInitialVelocity();

  auto estimated_vel = physics.getInitialVelocity();

  // 数値的に不安定な場合、最初の2点から推定される
  EXPECT_DOUBLE_EQ(estimated_vel.x(), 0.0);
  EXPECT_DOUBLE_EQ(estimated_vel.y(), 0.0);
  EXPECT_DOUBLE_EQ(estimated_vel.z(), 0.0);
}
}  // namespace crane
