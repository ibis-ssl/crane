// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cmath>
#include <crane_latency_estimator/latency_estimator.hpp>
#include <deque>
#include <functional>
#include <utility>

namespace crane
{
namespace
{
using Series = std::deque<std::pair<double, double>>;

constexpr double MAX_LAG_MS = 500.0;
constexpr double RESAMPLE_DT_MS = 10.0;
constexpr double MIN_CORRELATION = 0.3;
constexpr double MIN_CMD_STDDEV = 0.05;

// 周期を持たない信号（チャープ）。周期信号だと ±500 ms の中に相関のピークが複数でき、
// 遅延が一意に決まらない
double chirp(double t) { return 0.8 * std::sin(1.5 * t + 0.6 * t * t); }

// 10 ms 刻みで [0, count) 個の点を並べる
Series sample(const std::function<double(double)> & f, int count)
{
  Series s;
  for (int i = 0; i < count; ++i) {
    const double t = i * 0.01;
    s.emplace_back(t, f(t));
  }
  return s;
}

std::pair<double, double> estimate(
  const Series & cmd, const Series & obs, double min_correlation = MIN_CORRELATION)
{
  return LatencyEstimator::estimateLagMs(
    cmd, obs, MAX_LAG_MS, RESAMPLE_DT_MS, min_correlation, MIN_CMD_STDDEV);
}
}  // namespace

// 相関を重なりの点数で割るので、端の点が抜けるずれでは相関が 1 を超えることがあり、
// ピークが 1 刻み（10 ms）ずれうる。許容差はその分とる
TEST(EstimateLagMs, ObservationDelayedBy100msGivesPositive100ms)
{
  const auto cmd = sample(chirp, 500);
  const auto obs = sample([](double t) { return chirp(t - 0.1); }, 500);
  const auto [lag_ms, corr] = estimate(cmd, obs);
  EXPECT_NEAR(lag_ms, 100.0, 10.0);
  EXPECT_GT(corr, 0.99);
}

TEST(EstimateLagMs, IdenticalSignalsGiveZeroLag)
{
  const auto cmd = sample(chirp, 500);
  const auto [lag_ms, corr] = estimate(cmd, cmd);
  EXPECT_NEAR(lag_ms, 0.0, 10.0);
  EXPECT_GT(corr, 0.99);
}

TEST(EstimateLagMs, FewerThan10SamplesIsNotEstimated)
{
  const auto cmd = sample(chirp, 9);
  const auto obs = sample(chirp, 500);
  const auto [lag_ms, corr] = estimate(cmd, obs);
  EXPECT_TRUE(std::isnan(lag_ms));
  EXPECT_EQ(corr, 0.0);
}

TEST(EstimateLagMs, OverlapShorterThan20ResampledPointsIsNotEstimated)
{
  // 15 点 = 140 ms の重なりは、10 ms 刻みで 20 点に満たない
  const auto cmd = sample(chirp, 15);
  const auto obs = sample(chirp, 500);
  const auto [lag_ms, corr] = estimate(cmd, obs);
  EXPECT_TRUE(std::isnan(lag_ms));
  EXPECT_EQ(corr, 0.0);
}

TEST(EstimateLagMs, ConstantCommandIsNotEstimated)
{
  const auto cmd = sample([](double) { return 0.3; }, 500);
  const auto obs = sample(chirp, 500);
  const auto [lag_ms, corr] = estimate(cmd, obs);
  EXPECT_TRUE(std::isnan(lag_ms));
  EXPECT_EQ(corr, 0.0);
}

TEST(EstimateLagMs, CorrelationBelowThresholdReturnsNaNLagButKeepsCorrelation)
{
  const auto cmd = sample(chirp, 500);
  const auto obs = sample([](double t) { return chirp(t - 0.1); }, 500);
  // 相関は 1 をわずかに超えることはあっても 1.5 には届かないので、閾値 1.5 は必ず下回る
  const auto [lag_ms, corr] = estimate(cmd, obs, 1.5);
  EXPECT_TRUE(std::isnan(lag_ms));
  EXPECT_GT(corr, 0.99);
}

}  // namespace crane
