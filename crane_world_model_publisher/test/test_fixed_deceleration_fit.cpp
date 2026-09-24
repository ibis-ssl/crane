// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "crane_world_model_publisher/calibration/fixed_deceleration_fit.hpp"

namespace
{
// t = start, start+0.01, ... の n 点と、v = v0 - decel * t の速度
struct Samples
{
  std::vector<double> t;
  std::vector<double> v;
};

Samples makeLinear(size_t n, double start, double v0, double decel)
{
  Samples s;
  for (size_t i = 0; i < n; ++i) {
    const double t = start + 0.01 * static_cast<double>(i);
    s.t.push_back(t);
    s.v.push_back(v0 - decel * t);
  }
  return s;
}

double mean(const std::vector<double> & x)
{
  double sum = 0.0;
  for (double e : x) sum += e;
  return sum / x.size();
}
}  // namespace

TEST(FixedDecelerationFit, TrueDecelerationRecoversVelocityAtFirstSample)
{
  const auto s = makeLinear(20, 0.0, 3.0, 0.5);
  const auto fit = crane::fitFixedDeceleration(s.t, s.v, 0.5, 10);
  EXPECT_NEAR(fit.v0, 3.0, 1e-9);
  EXPECT_NEAR(fit.v0, s.v.front(), 1e-9);
  EXPECT_NEAR(fit.rmse, 0.0, 1e-9);
  EXPECT_TRUE(fit.accepted);
}

// v0 は候補 decel ごとに mean(v + decel * t) へ動く（自由回帰の切片なら 3.0 のまま動かない）
TEST(FixedDecelerationFit, MismatchedDecelerationShiftsV0ByMeanTime)
{
  const auto s = makeLinear(20, 0.0, 3.0, 0.5);
  const auto fit = crane::fitFixedDeceleration(s.t, s.v, 0.8, 10);

  const double mean_t = mean(s.t);
  double var_t = 0.0;
  for (double t : s.t) var_t += (t - mean_t) * (t - mean_t);
  var_t /= s.t.size();

  EXPECT_NEAR(fit.v0, 3.0 + 0.3 * mean_t, 1e-9);
  EXPECT_NEAR(fit.rmse, 0.3 * std::sqrt(var_t), 1e-9);
  EXPECT_TRUE(fit.accepted);
}

// v0 は最初の点ではなく t=0 の速度。時刻は軌道の最初の点を 0 として渡す前提
TEST(FixedDecelerationFit, V0IsVelocityAtTimeZero)
{
  const auto s = makeLinear(20, 0.5, 2.0, 0.5);
  const auto fit = crane::fitFixedDeceleration(s.t, s.v, 0.5, 10);
  EXPECT_NEAR(fit.v0, 2.0, 1e-9);
  EXPECT_NEAR(s.v.front(), 1.75, 1e-9);
}

TEST(FixedDecelerationFit, RejectsTrajectoryWithFewerThanMinPoints)
{
  const auto nine = makeLinear(9, 0.0, 3.0, 0.5);
  const auto rejected = crane::fitFixedDeceleration(nine.t, nine.v, 0.5, 10);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.v0, 0.0);
  EXPECT_EQ(rejected.rmse, 0.0);

  const auto ten = makeLinear(10, 0.0, 3.0, 0.5);
  EXPECT_TRUE(crane::fitFixedDeceleration(ten.t, ten.v, 0.5, 10).accepted);

  EXPECT_FALSE(crane::fitFixedDeceleration({}, {}, 0.5, 0).accepted);
}

TEST(FixedDecelerationFit, AcceptsOnlyV0Above0p1)
{
  const auto above = makeLinear(10, 0.0, 0.11, 0.5);
  EXPECT_TRUE(crane::fitFixedDeceleration(above.t, above.v, 0.5, 10).accepted);

  const auto below = makeLinear(10, 0.0, 0.09, 0.5);
  EXPECT_FALSE(crane::fitFixedDeceleration(below.t, below.v, 0.5, 10).accepted);
}

// 同じ軌道でも候補 decel によって v0 が 0.1 をまたぎ、採否が変わる
TEST(FixedDecelerationFit, AcceptanceNear0p1DependsOnCandidateDeceleration)
{
  const auto s = makeLinear(20, 0.0, 0.08, 0.5);  // mean(t) = 0.095

  const auto at_true = crane::fitFixedDeceleration(s.t, s.v, 0.5, 10);
  EXPECT_NEAR(at_true.v0, 0.08, 1e-9);
  EXPECT_FALSE(at_true.accepted);

  const auto at_larger = crane::fitFixedDeceleration(s.t, s.v, 0.8, 10);
  EXPECT_NEAR(at_larger.v0, 0.08 + 0.3 * 0.095, 1e-9);
  EXPECT_TRUE(at_larger.accepted);
}
