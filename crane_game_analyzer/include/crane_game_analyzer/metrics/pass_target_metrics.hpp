// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__PASS_TARGET_METRICS_HPP_
#define CRANE_GAME_ANALYZER__METRICS__PASS_TARGET_METRICS_HPP_

#include <crane_physics/slack_time_config.hpp>

#include "crane_game_analyzer/selection_hysteresis.hpp"
#include "metric_base.hpp"

namespace crane::metrics
{

/**
 * @brief パスターゲット選定メトリクス
 *
 * パススコアを算出し、ヒステリシス込みでpass_target_idを選定
 * 敵インターセプト評価（TTI）とシャドウヒューリスティックを統合
 */
class PassTargetMetric : public MetricBase
{
public:
  PassTargetMetric();

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override
  {
    return {MetricId::ONGOING_KICK};
  }

  auto compute(MetricContext & ctx) -> void override;
  auto visualize(MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer)
    -> void override;

  /**
   * @brief ヒステリシスパラメータを設定
   * @param min_hold_sec ターゲット保持の最短秒数
   * @param min_improvement 保持期間後の切替に必要な最小改善率
   */
  auto setHysteresisParams(double min_hold_sec, double min_improvement) -> void
  {
    pass_hysteresis_.setConfig({
      .min_hold_duration_sec = min_hold_sec,
      .min_improvement_ratio = min_improvement,
    });
  }

  auto setEnemySlackConfig(const SlackTimeConfig & config, double slack_scale = 0.5) -> void
  {
    enemy_slack_config_ = config;
    slack_scale_ = slack_scale;
  }

private:
  // パス起点の計算
  [[nodiscard]] auto computePassOrigin(MetricContext & ctx) const -> Point;

  // スコア計算
  [[nodiscard]] auto calcScore(
    MetricContext & ctx, const Point & pass_origin, const Point & p) const -> double;

  // ヒステリシス管理
  SelectionHysteresis<int> pass_hysteresis_{SelectionHysteresis<int>::Config{
    .min_hold_duration_sec = 0.5,
    .min_improvement_ratio = 0.2,
  }};

  // 敵インターセプト評価用パラメータ
  SlackTimeConfig enemy_slack_config_{
    .robot_max_acceleration = 3.0,  // 敵は自チームより高め（安全マージン）
    .robot_max_velocity = 5.5       // 敵は自チームより高め（安全マージン）
  };
  double slack_scale_ = 0.5;  // スコア正規化用
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__PASS_TARGET_METRICS_HPP_
