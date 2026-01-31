// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/pass_target_metrics.hpp"

#include <crane_physics/pass_evaluation.hpp>
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
  for (auto it = ctx.ball_history->rbegin(); it != ctx.ball_history->rend(); ++it) {
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
  double score = 1.0;

  // 距離（0〜4mで上昇）
  const double pass_distance = (p - pass_origin).norm();
  score += std::clamp(pass_distance * 0.5, 0.0, 2.0);

  // ゴール角度（敵ゴールに対する見通し）
  {
    auto [best_angle, goal_angle_width] = ctx.world_model->getLargestGoalAngleRangeFromPoint(p);
    score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);
  }

  // 自ゴールに対する危険度（大きいほど減点）
  {
    auto [best_angle, goal_angle_width] =
      ctx.world_model->getLargestGoalAngleRangeFromPoint(p, ctx.world_model->getOurGoalPosts(), {});
    score -= std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);
  }

  // 敵ゴールへの接近
  {
    double normed_distance_to_their_goal = ((p - ctx.world_model->getTheirGoalCenter()).norm() -
                                            (ctx.world_model->fieldSize().x() * 0.5)) /
                                           (ctx.world_model->fieldSize().x() * 0.5);
    score *= (1.0 - normed_distance_to_their_goal * 0.5);
  }

  constexpr double KICK_SPEED = 3.0;
  const Segment pass_line{pass_origin, p};
  const Vector2 pass_dir = p - pass_origin;
  const Vector2 ball_velocity =
    (pass_distance > 1e-6) ? Vector2(pass_dir / pass_distance * KICK_SPEED) : Vector2::Zero();

  auto calc_slack_time = [&](const auto & enemy) -> double {
    const auto closest = getClosestPointAndDistance(enemy->pose.pos, pass_line);
    const double ball_time = (closest.closest_point - pass_origin).norm() / KICK_SPEED;

    auto slack_result = ctx.world_model->getBallSlackTime(
      pass_origin, ball_velocity, ball_time, {enemy}, enemy_slack_config_);

    return slack_result.has_value() ? slack_result->slack_time : 1.0;
  };

  auto enemies = ctx.world_model->theirs().robotsWhere().available().get();
  auto slack_times_view = enemies | ranges::views::filter([&](const auto & enemy) {
                            // パス起点から近すぎる敵はチップで飛び越せるので除外
                            return enemy->getDistance(pass_origin) >= 1.0;
                          }) |
                          ranges::views::filter([&](const auto & enemy) {
                            // パス先より向こうにいる敵は除外
                            return pass_dir.dot(enemy->pose.pos - p) <= 0.0;
                          }) |
                          ranges::views::transform(calc_slack_time);

  const double worst_slack = ranges::empty(slack_times_view) ? 1.0 : ranges::min(slack_times_view);
  const double intercept_score = std::clamp(worst_slack / slack_scale_, 0.0, 1.0);

  score *= intercept_score;

  // シャドウ評価: パスライン上の敵による遮蔽効果
  const double shadow_score = evaluatePassShadow(pass_origin, p, enemies);
  score *= shadow_score;

  // ペナルティエリア内は無効
  if (ctx.world_model->point_checker.isPenaltyArea(p)) {
    score = 0.0;
  }

  return score;
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

    const rclcpp::Time now_time = ros_clock_.now();
    const bool hold_active = (now_time - last_switch_time_).seconds() < min_hold_duration_sec_;

    double prev_score = -1.0;
    if (last_pass_target_id_.has_value()) {
      auto it =
        ranges::find_if(ctx.analysis.pass_scores, [&](const crane_msgs::msg::FloatWithID & s) {
          return s.id == last_pass_target_id_.value();
        });
      if (it != ranges::end(ctx.analysis.pass_scores)) prev_score = it->value;
    }

    bool should_switch = true;
    if (last_pass_target_id_.has_value()) {
      if (best_id == last_pass_target_id_.value()) {
        should_switch = false;  // 同一なら切替不要
      } else if (hold_active) {
        if (prev_score >= 0.0) {
          should_switch = (best_score - prev_score) > min_improvement_margin_;
        } else {
          should_switch = false;  // 前回スコア不明
        }
      }
    }

    if (!last_pass_target_id_.has_value()) {
      last_pass_target_id_ = best_id;
      last_switch_time_ = now_time;
    } else if (should_switch) {
      last_pass_target_id_ = best_id;
      last_switch_time_ = now_time;
    }

    ctx.analysis.pass_target_id = last_pass_target_id_.value();
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
}

}  // namespace crane::metrics
