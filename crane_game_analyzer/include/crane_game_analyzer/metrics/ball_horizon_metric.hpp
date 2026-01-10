// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__BALL_HORIZON_METRIC_HPP_
#define CRANE_GAME_ANALYZER__METRICS__BALL_HORIZON_METRIC_HPP_

#include "metric_base.hpp"

namespace crane::metrics
{

/**
 * @brief ボールライン長メトリクス
 *
 * ボール軌道に対する敵ロボットの干渉距離を計算
 * - ボールから3秒後までの軌道を計算
 * - 敵ロボットとの最短距離を求める
 * - 0.5m以内の敵がいる場合、その地点までの距離を返す
 */
class BallHorizonMetric : public MetricBase
{
public:
  BallHorizonMetric();

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override { return {}; }

  auto compute(MetricContext & ctx) -> void override;
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__BALL_HORIZON_METRIC_HPP_
