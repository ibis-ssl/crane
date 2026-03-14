// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__METRICS__THREAT_METRICS_HPP_
#define CRANE_GAME_ANALYZER__METRICS__THREAT_METRICS_HPP_

#include "crane_game_analyzer/threat_evaluator.hpp"
#include "metric_base.hpp"

namespace crane::metrics
{

/**
 * @brief ボール脅威メトリクス
 *
 * ボール位置からゴールへの脅威を評価
 */
class BallThreatMetric : public MetricBase
{
public:
  explicit BallThreatMetric(std::shared_ptr<ThreatEvaluator> evaluator);

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override { return {}; }

  auto compute(MetricContext & ctx) -> void override;

  auto visualize(MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer)
    -> void override;

  /// 他のメトリクスが参照できるよう、最後の計算結果を公開
  [[nodiscard]] auto getLastBallThreat() const -> const BallThreat & { return last_ball_threat_; }

private:
  std::shared_ptr<ThreatEvaluator> evaluator_;
  BallThreat last_ball_threat_;
};

/**
 * @brief ロボット脅威メトリクス
 *
 * 敵ロボットの脅威度を評価（優先度順）
 * BALL_THREATに依存
 */
class RobotThreatsMetric : public MetricBase
{
public:
  RobotThreatsMetric(
    std::shared_ptr<BallThreatMetric> ball_threat_metric,
    std::shared_ptr<ThreatEvaluator> evaluator);

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override
  {
    return {MetricId::BALL_THREAT};
  }

  auto compute(MetricContext & ctx) -> void override;

  auto visualize(MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer)
    -> void override;

  /// 他のメトリクスが参照できるよう、最後の計算結果を公開
  [[nodiscard]] auto getLastRobotThreats() const -> const std::vector<RobotThreat> &
  {
    return last_robot_threats_;
  }

private:
  std::shared_ptr<BallThreatMetric> ball_threat_metric_;
  std::shared_ptr<ThreatEvaluator> evaluator_;
  std::vector<RobotThreat> last_robot_threats_;

  /// 脅威度に応じた色を返すヘルパー
  static auto threatToColor(double threat_rating) -> std::string;
};

/**
 * @brief 推奨守備者数メトリクス
 *
 * ボールとロボット脅威から推奨守備者数を計算
 * BALL_THREAT, ROBOT_THREATSに依存
 */
class RecommendedDefendersMetric : public MetricBase
{
public:
  RecommendedDefendersMetric(
    std::shared_ptr<BallThreatMetric> ball_threat_metric,
    std::shared_ptr<RobotThreatsMetric> robot_threats_metric,
    std::shared_ptr<ThreatEvaluator> evaluator);

  [[nodiscard]] auto getDependencies() const -> std::vector<MetricId> override
  {
    return {MetricId::BALL_THREAT, MetricId::ROBOT_THREATS};
  }

  auto compute(MetricContext & ctx) -> void override;

  auto visualize(MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer)
    -> void override;

private:
  std::shared_ptr<BallThreatMetric> ball_threat_metric_;
  std::shared_ptr<RobotThreatsMetric> robot_threats_metric_;
  std::shared_ptr<ThreatEvaluator> evaluator_;
};

}  // namespace crane::metrics

#endif  // CRANE_GAME_ANALYZER__METRICS__THREAT_METRICS_HPP_
