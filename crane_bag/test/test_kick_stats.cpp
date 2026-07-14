// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include "bag_kick_stats.hpp"

namespace cb = crane::bag;

namespace
{

cb::TimestampedMsg<cb::KickPredictionTraceData> trace(
  double kick_power, bool is_chip, double pred_speed, double actual_speed, double pred_dist,
  double actual_dist)
{
  cb::KickPredictionTraceData tr;
  tr.has_prediction = true;
  tr.kick_power = kick_power;
  tr.is_chip_kick = is_chip;
  tr.predicted_ball_speed = pred_speed;
  tr.predicted_stop_distance = pred_dist;
  tr.has_actual = true;
  tr.actual_ball_speed = actual_speed;
  tr.actual_stop_distance = actual_dist;
  tr.speed_error = actual_speed - pred_speed;
  tr.speed_error_percent = pred_speed > 0 ? (actual_speed - pred_speed) / pred_speed * 100.0 : 0;
  tr.distance_error = actual_dist - pred_dist;
  tr.distance_error_percent = pred_dist > 0 ? (actual_dist - pred_dist) / pred_dist * 100.0 : 0;
  return {0, tr};
}

}  // namespace

TEST(KickStats, EmptyInput)
{
  auto s = cb::compute_kick_stats({});
  EXPECT_EQ(s.total, 0u);
  EXPECT_EQ(s.with_actual, 0u);
  EXPECT_EQ(s.straight.count, 0u);
}

TEST(KickStats, GroupsAndBias)
{
  std::vector<cb::TimestampedMsg<cb::KickPredictionTraceData>> traces;
  // straight: 予測4.0に対し実測が常に10%低い → bias -10%
  traces.push_back(trace(0.5, false, 4.0, 3.6, 8.0, 7.2));
  traces.push_back(trace(0.5, false, 4.0, 3.6, 8.0, 7.2));
  // chip: 距離が20%長い → bias +20%
  traces.push_back(trace(0.8, true, 0.0, 0.0, 3.0, 3.6));
  // actual なしは with_actual に数えない
  {
    cb::KickPredictionTraceData tr;
    tr.has_prediction = true;
    traces.push_back({0, tr});
  }

  auto s = cb::compute_kick_stats(traces);
  EXPECT_EQ(s.total, 4u);
  EXPECT_EQ(s.with_actual, 3u);
  ASSERT_EQ(s.straight.count, 2u);
  ASSERT_EQ(s.chip.count, 1u);
  EXPECT_NEAR(s.straight.speed_bias_percent, -10.0, 1e-6);
  EXPECT_NEAR(s.straight.speed_abs_p50, 10.0, 1e-6);
  EXPECT_NEAR(s.straight.dist_bias_percent, -10.0, 1e-6);
  EXPECT_NEAR(s.chip.dist_bias_percent, 20.0, 1e-6);

  // power bin: straight は 0.5 → bin 0.5-0.6
  ASSERT_EQ(s.straight.power_bins.size(), 1u);
  EXPECT_NEAR(s.straight.power_bins[0].power_lo, 0.5, 1e-9);
  EXPECT_EQ(s.straight.power_bins[0].count, 2u);
  EXPECT_NEAR(s.straight.power_bins[0].mean_actual_speed, 3.6, 1e-6);
}

TEST(KickStats, PercentileNearestRank)
{
  std::vector<cb::TimestampedMsg<cb::KickPredictionTraceData>> traces;
  // |speed error %| = 5, 10, 15, 20, 25 の5点
  for (int i = 1; i <= 5; ++i) {
    const double err = 0.05 * i;
    traces.push_back(trace(0.5, false, 4.0, 4.0 * (1.0 + err), 8.0, 8.0));
  }
  auto s = cb::compute_kick_stats(traces);
  ASSERT_EQ(s.straight.count, 5u);
  EXPECT_NEAR(s.straight.speed_abs_p50, 15.0, 1e-6);  // 5点の nearest-rank 50% → 3番目
  EXPECT_NEAR(s.straight.speed_abs_p90, 25.0, 1e-6);  // 90% → 5番目
  EXPECT_NEAR(s.straight.speed_bias_percent, 15.0, 1e-6);
}
