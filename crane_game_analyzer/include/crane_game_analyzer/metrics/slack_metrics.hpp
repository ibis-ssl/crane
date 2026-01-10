// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__SLACK_METRICS_HPP_
#define CRANE_GAME_ANALYZER__METRICS__SLACK_METRICS_HPP_

#include "metric_base.hpp"

namespace crane::metrics
{

/**
 * @brief 味方ロボットのSlack時間メトリクス
 *
 * 各味方ロボットのボールインターセプト余裕時間を計算
 * - min_slack: 最も早くボールに到達できる地点
 * - max_slack: 最も余裕がある地点
 */
class OurSlackMetric : public MetricBase
{
public:
  OurSlackMetric();

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override { return {}; }

  auto compute(MetricContext & ctx) -> void override;
};

/**
 * @brief 敵ロボットのSlack時間メトリクス
 *
 * 各敵ロボットのボールインターセプト余裕時間を計算
 */
class TheirSlackMetric : public MetricBase
{
public:
  TheirSlackMetric();

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override { return {}; }

  auto compute(MetricContext & ctx) -> void override;
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__SLACK_METRICS_HPP_
