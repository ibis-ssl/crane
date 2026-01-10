// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__ATTACKER_METRICS_HPP_
#define CRANE_GAME_ANALYZER__METRICS__ATTACKER_METRICS_HPP_

#include "metric_base.hpp"

namespace crane::metrics
{

/**
 * @brief 推奨アタッカーメトリクス
 *
 * ボール距離とSlack時間を考慮して推奨Attackerを決定
 * OUR_SLACK, BALL_THREATに依存
 */
class AttackerCandidateMetric : public MetricBase
{
public:
  AttackerCandidateMetric();

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override
  {
    return {MetricId::OUR_SLACK, MetricId::BALL_THREAT};
  }

  auto compute(MetricContext & ctx) -> void override;

private:
  // ヒステリシス用の内部状態
  int last_attacker_id_ = -1;
  rclcpp::Time last_switch_time_{static_cast<int64_t>(0), RCL_ROS_TIME};
  rclcpp::Clock ros_clock_{RCL_ROS_TIME};

  static constexpr double MIN_HOLD_DURATION_SEC = 0.5;
  static constexpr double MIN_IMPROVEMENT_MARGIN = 0.2;
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__ATTACKER_METRICS_HPP_
