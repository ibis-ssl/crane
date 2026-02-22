// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__ONGOING_KICK_METRIC_HPP_
#define CRANE_GAME_ANALYZER__METRICS__ONGOING_KICK_METRIC_HPP_

#include "metric_base.hpp"

namespace crane::metrics
{

/**
 * @brief 進行中キック検出メトリクス
 *
 * KickEventDetectorの結果をGameAnalysis.ongoing_kickへ反映する。
 * 他メトリクスは依存関係でこの結果を参照できる。
 */
class OngoingKickMetric : public MetricBase
{
public:
  OngoingKickMetric();

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override { return {}; }

  auto compute(MetricContext & ctx) -> void override;
  auto visualize(MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer)
    -> void override;
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__ONGOING_KICK_METRIC_HPP_
