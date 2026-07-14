// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/pass_target_metrics.hpp"

#include <crane_msg_wrappers/pass_rating.hpp>
#include <cstdio>
#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/functional/comparisons.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

namespace crane::metrics
{

PassTargetMetric::PassTargetMetric() : MetricBase(MetricId::PASS_TARGET, "PassTarget") {}

auto PassTargetMetric::computePassOrigin(MetricContext & ctx) const -> Point
{
  const auto & ball = ctx.world_model->ball();
  // 検出かつ停止
  if (ball.isStopped() && ball.detected) {
    return ball.pos;
  }
  // 検出かつ移動
  if (ball.detected && ball.isMoving()) {
    return ball.getPredictedPosition(std::min(ball.getStopTime(), 1.0));
  }
  // 履歴から直近検出
  for (auto it = ctx.ball_history->begin(); it != ctx.ball_history->end(); ++it) {
    if (it->detected) {
      return Point(it->position.x, it->position.y);
    }
  }
  // キック起点
  if (not ctx.analysis.ongoing_kick.empty()) {
    const auto & k = ctx.analysis.ongoing_kick.front();
    return Point(k.origin_x, k.origin_y);
  }
  // フォールバック
  return ball.pos;
}

auto PassTargetMetric::calcScore(
  MetricContext & ctx, const Point & pass_origin, const Point & p) const -> double
{
  // 評価は crane_msg_wrappers の ratePassCandidate に一元化（挙動保存）。
  // 選定ゲート（min_pass_score_）とヒステリシスは compute() 側に残す。
  return ratePassCandidate(
           ctx.world_model, pass_origin, p,
           PassRatingConfig{.slack_scale = slack_scale_, .enemy_slack = enemy_slack_config_})
    .score;
}

auto PassTargetMetric::compute(MetricContext & ctx) -> void
{
  // パス起点の決定
  const Point pass_origin = computePassOrigin(ctx);

  // 候補のスコア算出
  auto our_robots = ctx.world_model->ours().robotsWhere().available().excludeGoalie().get();
  auto score_with_bots =
    our_robots | ranges::views::filter([&](const auto & robot) {
      return robot->id != ctx.world_model->getOurGoalieId() &&
             robot->pose.pos.x() * ctx.world_model->getOurSideSign() <= 0.0 &&
             !ctx.world_model->point_checker.isPenaltyArea(robot->pose.pos);
    }) |
    ranges::views::transform([&](const auto & robot) {
      return std::make_pair(robot, calcScore(ctx, pass_origin, robot->pose.pos));
    }) |
    ranges::to<std::vector>();

  ranges::sort(score_with_bots, ranges::greater{}, [](const auto & p) { return p.second; });

  ctx.analysis.pass_scores.clear();
  ctx.analysis.pass_scores.reserve(score_with_bots.size());
  for (const auto & [robot, score] : score_with_bots) {
    crane_msgs::msg::FloatWithID msg;
    msg.set__id(robot->id).set__value(score);
    ctx.analysis.pass_scores.push_back(msg);
  }

  // ヒステリシスによるターゲット選定
  ctx.analysis.pass_target_id = -1;
  if (!ctx.analysis.pass_scores.empty()) {
    const auto & best = ctx.analysis.pass_scores.front();
    const int best_id = static_cast<int>(best.id);
    const double best_score = static_cast<double>(best.value);

    // 最低スコア閾値チェック: パス品質が低すぎる場合はパス不可
    if (best_score < min_pass_score_) {
      pass_hysteresis_.reset();
      return;
    }

    double prev_score = 0.0;
    const auto prev_id = pass_hysteresis_.currentId();
    if (prev_id.has_value()) {
      auto it = ranges::find_if(
        ctx.analysis.pass_scores,
        [&](const crane_msgs::msg::FloatWithID & s) { return s.id == prev_id.value(); });
      if (it != ranges::end(ctx.analysis.pass_scores)) prev_score = it->value;
    }

    pass_hysteresis_.shouldSwitch(best_id, best_score, prev_score);
    ctx.analysis.pass_target_id = pass_hysteresis_.currentId().value_or(-1);
  }
}

auto PassTargetMetric::visualize(
  MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer) -> void
{
  if (ctx.analysis.pass_target_id < 0) {
    return;
  }

  const Point pass_origin = computePassOrigin(ctx);
  auto receiver = ctx.world_model->getOurRobot(static_cast<uint8_t>(ctx.analysis.pass_target_id));

  if (ctx.world_model->point_checker.isFieldInside(pass_origin)) {
    visualizer->drawLine(pass_origin, receiver->pose.pos, "lime", 40, 0.8);
    visualizer->drawCircle(pass_origin, 0.12, "lime", 10);
  }
  visualizer->drawStyledCircle(receiver->pose.pos, 0.5, "lime", 0.15, "lime", 1.0, 18);
  visualizer->drawText(
    Point(receiver->pose.pos.x(), receiver->pose.pos.y() + 0.35),
    std::string("PASS TARGET #") + std::to_string(receiver->id), "lime", 110.0, "middle");

  // スコア内訳（M1-5）: 選定された受け手のパス評価を分解表示
  const auto rating = ratePassCandidate(
    ctx.world_model, pass_origin, receiver->pose.pos,
    PassRatingConfig{.slack_scale = slack_scale_, .enemy_slack = enemy_slack_config_});
  char breakdown[192];
  std::snprintf(
    breakdown, sizeof(breakdown),
    "score %.2f = dist %.2f +goal %.2f -own %.2f *tgoal %.2f *icpt %.2f *shd %.2f", rating.score,
    rating.distance_factor, rating.goal_angle_bonus, rating.own_goal_penalty,
    rating.their_goal_factor, rating.intercept_score, rating.shadow_score);
  visualizer->drawText(
    Point(receiver->pose.pos.x(), receiver->pose.pos.y() - 0.45), std::string(breakdown), "lime",
    60.0, "middle");
}

}  // namespace crane::metrics
