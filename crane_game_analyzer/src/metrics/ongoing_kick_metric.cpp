// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/ongoing_kick_metric.hpp"

#include "crane_game_analyzer/kick_event_detector.hpp"

namespace crane::metrics
{

OngoingKickMetric::OngoingKickMetric() : MetricBase(MetricId::ONGOING_KICK, "OngoingKick") {}

auto OngoingKickMetric::compute(MetricContext & ctx) -> void
{
  ctx.analysis.ongoing_kick.clear();

  if (!ctx.kick_event_detector) {
    return;
  }

  // 可視化はこのmetricのvisualize()に集約するため、detector更新時は描画しない
  ctx.kick_event_detector->update(*ctx.world_model, nullptr);
  if (auto kick = ctx.kick_event_detector->getOnGoingKick(); kick.has_value()) {
    ctx.analysis.ongoing_kick.push_back(*kick);
  }
}

auto OngoingKickMetric::visualize(
  MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer) -> void
{
  if (!visualizer || ctx.analysis.ongoing_kick.empty()) {
    return;
  }

  const auto & kick = ctx.analysis.ongoing_kick.front();
  Point origin(kick.origin_x, kick.origin_y);
  Point ball_pos = ctx.world_model->ball().pos;

  visualizer->drawLine(origin, ball_pos, "red", 200, 0.3);
  visualizer->drawCircle(origin, 0.08, "red", 10);
}

}  // namespace crane::metrics
