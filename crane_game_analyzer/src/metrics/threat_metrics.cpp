// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/threat_metrics.hpp"

#include <algorithm>

namespace crane::metrics
{

// BallThreatMetric実装

BallThreatMetric::BallThreatMetric(std::shared_ptr<ThreatEvaluator> evaluator)
: MetricBase(MetricId::BALL_THREAT, "BallThreat"), evaluator_(std::move(evaluator))
{
}

auto BallThreatMetric::compute(MetricContext & ctx) -> void
{
  last_ball_threat_ = evaluator_->calculateBallThreat(*ctx.world_model);
  ctx.analysis.ball_threat = evaluator_->toThreatInfoMsg(last_ball_threat_);
}

auto BallThreatMetric::visualize(
  MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer) -> void
{
  (void)ctx;

  // ボール脅威ライン（オレンジ）
  visualizer->line()
    .fromSegment(last_ball_threat_.threat_line)
    .stroke("orange", 0.9)
    .strokeWidth(3)
    .build();

  // ボール脅威の防御ライン（シアン）
  if (last_ball_threat_.protection_line) {
    visualizer->line()
      .fromSegment(*last_ball_threat_.protection_line)
      .stroke("cyan", 0.8)
      .strokeWidth(2)
      .build();

    // 防御ライン端点のマーカー
    visualizer->drawStyledCircle(
      last_ball_threat_.protection_line->first, 0.03, "cyan", 0.5, "cyan", 1.0, 2);
    visualizer->drawStyledCircle(
      last_ball_threat_.protection_line->second, 0.03, "cyan", 0.5, "cyan", 1.0, 2);
  }
}

// RobotThreatsMetric実装

RobotThreatsMetric::RobotThreatsMetric(
  std::shared_ptr<BallThreatMetric> ball_threat_metric, std::shared_ptr<ThreatEvaluator> evaluator)
: MetricBase(MetricId::ROBOT_THREATS, "RobotThreats"),
  ball_threat_metric_(ball_threat_metric),
  evaluator_(std::move(evaluator))
{
}

auto RobotThreatsMetric::compute(MetricContext & ctx) -> void
{
  const auto & ball_threat = ball_threat_metric_->getLastBallThreat();
  last_robot_threats_ = evaluator_->calculateRobotThreats(*ctx.world_model, ball_threat);

  ctx.analysis.robot_threats.clear();
  for (const auto & threat : last_robot_threats_) {
    ctx.analysis.robot_threats.push_back(evaluator_->toThreatInfoMsg(threat));
  }
}

auto RobotThreatsMetric::visualize(
  MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer) -> void
{
  // ロボット脅威（上位5つを可視化）
  int vis_count = 0;
  for (const auto & threat : last_robot_threats_) {
    if (vis_count >= 5) break;

    // 脅威度に応じたグラデーション色
    std::string color = threatToColor(threat.threat_rating);

    // 線の太さも脅威度に応じて変化 (1.0 - 4.0)
    double line_width = 1.0 + threat.threat_rating * 3.0;

    // 不透明度も脅威度に応じて変化 (0.4 - 1.0)
    double opacity = 0.4 + threat.threat_rating * 0.6;

    // 脅威ライン
    visualizer->line()
      .fromSegment(threat.threat_line)
      .stroke(color, opacity)
      .strokeWidth(line_width)
      .build();

    // 脅威スコア表示
    std::string score_text = std::to_string(threat.threat_rating).substr(0, 4);
    visualizer->drawCenteredLabel(threat.robot->pose.pos + Vector2(0, 0.15), score_text, color, 30);

    // 順位表示
    std::string rank_text = "#" + std::to_string(vis_count + 1);
    visualizer->drawCenteredLabel(
      threat.robot->pose.pos + Vector2(-0.12, 0.15), rank_text, color, 20);

    // 防御ライン（存在する場合）
    if (threat.protection_line) {
      visualizer->line()
        .fromSegment(*threat.protection_line)
        .stroke("cyan", 0.6)
        .strokeWidth(1.5)
        .build();
    }

    vis_count++;
  }

  // 上位脅威のリダイレクト角度を可視化（上位2つのみ）
  if (!last_robot_threats_.empty()) {
    for (size_t i = 0; i < std::min(size_t(2), last_robot_threats_.size()); ++i) {
      const auto & threat = last_robot_threats_[i];
      Point ball_pos = ctx.world_model->ball().pos;
      Point goal_center = ctx.world_model->getOurGoalCenter();
      Point threat_pos = threat.robot->pose.pos;

      // ボール→脅威→ゴール のリダイレクト角度を弧で表示
      Vector2 from_ball = (threat_pos - ball_pos).normalized();
      Vector2 to_goal = (goal_center - threat_pos).normalized();

      double angle1 = std::atan2(-from_ball.y(), -from_ball.x());
      double angle2 = std::atan2(to_goal.y(), to_goal.x());

      // 角度が大きすぎる場合はスキップ
      double angle_diff = std::abs(angle2 - angle1);
      if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
      if (angle_diff < M_PI) {
        std::string arc_color = threatToColor(threat.threat_rating);
        visualizer->arc(
          threat_pos, 0.15, std::min(angle1, angle2), std::max(angle1, angle2), arc_color, 1.5, 8);
      }
    }
  }
}

auto RobotThreatsMetric::threatToColor(double threat_rating) -> std::string
{
  // 脅威度に応じた色（緑→黄→赤）
  if (threat_rating < 0.3) {
    return "green";
  } else if (threat_rating < 0.6) {
    return "yellow";
  } else {
    return "red";
  }
}

// RecommendedDefendersMetric実装

RecommendedDefendersMetric::RecommendedDefendersMetric(
  std::shared_ptr<BallThreatMetric> ball_threat_metric,
  std::shared_ptr<RobotThreatsMetric> robot_threats_metric,
  std::shared_ptr<ThreatEvaluator> evaluator)
: MetricBase(MetricId::RECOMMENDED_DEFENDERS, "RecommendedDefenders"),
  ball_threat_metric_(ball_threat_metric),
  robot_threats_metric_(robot_threats_metric),
  evaluator_(std::move(evaluator))
{
}

auto RecommendedDefendersMetric::compute(MetricContext & ctx) -> void
{
  const auto & ball_threat = ball_threat_metric_->getLastBallThreat();
  const auto & robot_threats = robot_threats_metric_->getLastRobotThreats();

  int available_robots =
    static_cast<int>(ctx.world_model->ours().robotsWhere().available().get().size());
  ctx.analysis.recommended_num_defenders = static_cast<uint8_t>(
    evaluator_->calculateRecommendedDefenders(ball_threat, robot_threats, available_robots));
}

auto RecommendedDefendersMetric::visualize(
  MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer) -> void
{
  // 推奨守備者数を画面左上に表示
  std::string def_text = "DEF:" + std::to_string(ctx.analysis.recommended_num_defenders);
  visualizer->text().viewBoxPosition(3, -97).text(def_text).fill("cyan").fontSize(100).build();
}

}  // namespace crane::metrics
