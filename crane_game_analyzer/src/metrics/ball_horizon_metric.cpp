// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/ball_horizon_metric.hpp"

#include <crane_geometry/geometry_operations.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

namespace crane::metrics
{

BallHorizonMetric::BallHorizonMetric() : MetricBase(MetricId::BALL_HORIZON, "BallHorizon") {}

auto BallHorizonMetric::compute(MetricContext & ctx) -> void
{
  const auto & ball = ctx.world_model->ball();

  // ボールラインの長さを計算
  ctx.analysis.ball_horizon = [&]() {
    Segment ball_line = ball.getTrajectorySegmentByTime(3.0);
    auto robots = ctx.world_model->theirs().getAvailableRobots();
    auto ball_line_lengths =
      robots |
      ranges::views::transform(
        [&](const auto & robot) { return getClosestPointAndDistance(ball_line, robot->pose.pos); })
      // 距離が0.5m以下のものを抽出
      | ranges::views::filter([](const ClosestPoint & pair) { return pair.distance < 0.5; })
      // ball.posとの距離を計算
      | ranges::views::transform([&](const ClosestPoint & pair) -> double {
          return (pair.closest_point - ball.pos).norm();
        });
    return ranges::empty(ball_line_lengths) ? 10.0 : ranges::min(ball_line_lengths);
  }();
}

}  // namespace crane::metrics
