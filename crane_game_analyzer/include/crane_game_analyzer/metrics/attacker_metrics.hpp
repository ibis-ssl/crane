// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__ATTACKER_METRICS_HPP_
#define CRANE_GAME_ANALYZER__METRICS__ATTACKER_METRICS_HPP_

#include "crane_game_analyzer/selection_hysteresis.hpp"
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
    return {MetricId::OUR_SLACK, MetricId::THEIR_SLACK, MetricId::BALL_THREAT};
  }

  auto compute(MetricContext & ctx) -> void override;

private:
  // ヒステリシス管理
  SelectionHysteresis<int> attacker_hysteresis_{SelectionHysteresis<int>::Config{
    .min_hold_duration_sec = 0.5,   // 200ms → 500ms: パスカットウィンドウ内の振動抑制
    .min_improvement_ratio = 0.15,  // 1% → 15%: 僅差での無意味な切替を防止
    .emergency_switch_ratio = 1.5,  // 3% → 50%: 明らかに優位な場合のみ即切替
    .force_switch_timeout = 3.0,
    .absolute_threshold = 2.0}};

  // EMAスコア管理用マップ
  std::unordered_map<uint8_t, double> ema_scores_;

  // EMAスムージング係数
  static constexpr double EMA_ALPHA = 0.3;  // 新スコア30%、旧70%でスコア安定化
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__ATTACKER_METRICS_HPP_
