// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <vector>

#include "crane_world_model_publisher/calibration/initial_velocity_fit.hpp"

namespace
{
// t = t_offset + 0, 0.1, ..., 0.9 [s] に、最初の点で v0 から decel で減速する速度列を作る
struct Samples
{
  std::vector<double> t;
  std::vector<double> v;
};

Samples makeLinear(double v0, double decel, double t_offset = 0.0)
{
  Samples s;
  for (int i = 0; i < 10; ++i) {
    const double t = t_offset + 0.1 * i;
    s.t.push_back(t);
    s.v.push_back(v0 - decel * (t - t_offset));
  }
  return s;
}
}  // namespace

TEST(InitialVelocityFit, KnownDecelerationRecoversInitialVelocity)
{
  const auto s = makeLinear(3.0, 0.5);
  const auto fit = crane::fitInitialVelocityWithFixedDeceleration(s.t, s.v, 0.5);
  EXPECT_NEAR(fit.initial_velocity, 3.0, 1e-9);
  EXPECT_NEAR(fit.r_squared, 1.0, 1e-9);
  EXPECT_NEAR(fit.residual_rms, 0.0, 1e-9);
}

TEST(InitialVelocityFit, UsesGivenDecelerationInsteadOfFreeSlope)
{
  // 実際の傾きは 0.5 だが 0.3 で固定すると v0 = 3.0 - 0.2 * mean(t)（mean(t) = 0.45）
  const auto s = makeLinear(3.0, 0.5);
  const auto fit = crane::fitInitialVelocityWithFixedDeceleration(s.t, s.v, 0.3);
  EXPECT_NEAR(fit.initial_velocity, 3.0 - 0.2 * 0.45, 1e-9);
  EXPECT_LT(fit.r_squared, 1.0);
  EXPECT_GT(fit.residual_rms, 0.0);
}

TEST(InitialVelocityFit, InitialVelocityIsSpeedAtTimeZero)
{
  // 同じ速度列でも t が 1.0 秒ずれていれば、v0 は t=0 まで外挿した decel * 1.0 だけ大きい
  const auto s = makeLinear(3.0, 0.5, 1.0);
  const auto fit = crane::fitInitialVelocityWithFixedDeceleration(s.t, s.v, 0.5);
  EXPECT_NEAR(fit.initial_velocity, 3.5, 1e-9);
}

TEST(InitialVelocityFit, FewPoints)
{
  // 2 点: v0 = mean(2.0 + 0, 1.8 + 0.05) = 1.925、残差 ±0.075、ss_tot = 0.02
  const auto two = crane::fitInitialVelocityWithFixedDeceleration({0.0, 0.1}, {2.0, 1.8}, 0.5);
  EXPECT_NEAR(two.initial_velocity, 1.925, 1e-9);
  EXPECT_NEAR(two.r_squared, 1.0 - 0.01125 / 0.02, 1e-9);
  EXPECT_NEAR(two.residual_rms, 0.075, 1e-9);

  // 1 点: ss_tot = 0 なので R² は 0
  const auto one = crane::fitInitialVelocityWithFixedDeceleration({0.2}, {1.0}, 0.5);
  EXPECT_NEAR(one.initial_velocity, 1.1, 1e-9);
  EXPECT_DOUBLE_EQ(one.r_squared, 0.0);
  EXPECT_NEAR(one.residual_rms, 0.0, 1e-9);

  const auto none = crane::fitInitialVelocityWithFixedDeceleration({}, {}, 0.5);
  EXPECT_DOUBLE_EQ(none.initial_velocity, 0.0);
  EXPECT_DOUBLE_EQ(none.r_squared, 0.0);
  EXPECT_DOUBLE_EQ(none.residual_rms, 0.0);
}

TEST(InitialVelocityFit, RSquaredBecomesNegativeWhenSlopeContradictsDeceleration)
{
  // 加速している軌道に正の減速度を当てはめると、平均値より悪いので R² < 0
  const auto s = makeLinear(1.0, -0.5);
  const auto fit = crane::fitInitialVelocityWithFixedDeceleration(s.t, s.v, 0.5);
  EXPECT_LT(fit.r_squared, 0.0);
}

TEST(InitialVelocityFit, ConstantVelocityGivesZeroRSquared)
{
  const auto s = makeLinear(2.0, 0.0);

  // 全点同じ速度だと ss_tot = 0 なので、当てはまりが完全でも R² は 0
  const auto exact = crane::fitInitialVelocityWithFixedDeceleration(s.t, s.v, 0.0);
  EXPECT_NEAR(exact.initial_velocity, 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(exact.r_squared, 0.0);
  EXPECT_NEAR(exact.residual_rms, 0.0, 1e-9);

  const auto decel = crane::fitInitialVelocityWithFixedDeceleration(s.t, s.v, 0.5);
  EXPECT_NEAR(decel.initial_velocity, 2.0 + 0.5 * 0.45, 1e-9);
  EXPECT_DOUBLE_EQ(decel.r_squared, 0.0);
  EXPECT_GT(decel.residual_rms, 0.0);
}
