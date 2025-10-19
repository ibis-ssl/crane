// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/pass_target_selector.hpp"

#include <algorithm>

namespace crane
{

auto PassTargetSelector::computePassOrigin(
  const WorldModelWrapper::SharedPtr & world_model,
  const std::deque<crane_msgs::msg::BallInfo> & ball_history,
  const crane_msgs::msg::GameAnalysis & analysis_msg) const -> Point
{
  const auto & ball = world_model->ball();
  // 検出かつ停止
  if (ball.isStopped() && ball.detected) {
    return ball.pos;
  }
  // 検出かつ移動
  if (ball.detected && ball.isMoving()) {
    return ball.getPredictedPosition(std::min(ball.getStopTime(), 1.0));
  }
  // 履歴から直近検出
  for (auto it = ball_history.rbegin(); it != ball_history.rend(); ++it) {
    if (it->detected) {
      return Point(it->position.x, it->position.y);
    }
  }
  // キック起点
  if (not analysis_msg.ongoing_kick.empty()) {
    const auto & k = analysis_msg.ongoing_kick.front();
    return Point(k.origin_x, k.origin_y);
  }
  // フォールバック
  return ball.pos;
}

auto PassTargetSelector::calcScore(
  const WorldModelWrapper::SharedPtr & world_model, const Point & pass_origin, const Point & p)
  -> double
{
  Segment ball_to_target{pass_origin, p};
  double score = 1.0;
  // 距離（0〜4mで上昇）
  score += std::clamp((p - pass_origin).norm() * 0.5, 0.0, 2.0);
  // ゴール角度（敵ゴールに対する見通し）
  {
    auto [best_angle, goal_angle_width] = world_model->getLargestGoalAngleRangeFromPoint(p);
    score += std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);
  }
  // 自ゴールに対する危険度（大きいほど減点）
  {
    auto [best_angle, goal_angle_width] =
      world_model->getLargestGoalAngleRangeFromPoint(p, world_model->getOurGoalPosts(), {});
    score -= std::clamp(goal_angle_width / (M_PI / 12.), 0.0, 0.5);
  }
  // 敵ゴールへの接近
  {
    double normed_distance_to_their_goal =
      ((p - world_model->getTheirGoalCenter()).norm() - (world_model->fieldSize().x() * 0.5)) /
      (world_model->fieldSize().x() * 0.5);
    score *= (1.0 - normed_distance_to_their_goal * 0.5);
  }
  // パスカット
  std::vector<std::pair<double, std::shared_ptr<RobotInfo>>> enemys_with_angle_diff;
  for (const auto & enemy : world_model->theirs().getAvailableRobots()) {
    if (enemy->getDistance(pass_origin) < 1.0) {
      continue;  // チップキックで飛び越せる近くのロボットは対象外
    }
    if ((p - pass_origin).dot(enemy->pose.pos - p) > 0.0) {
      continue;  // パス先より向こうにいるロボットは対象外
    }
    double angle_diff = getAngleDiff(getAngle(p - pass_origin), getAngle(enemy->pose.pos));
    enemys_with_angle_diff.emplace_back(std::abs(angle_diff), enemy);
  }
  if (!enemys_with_angle_diff.empty()) {
    const auto & best_intercepter = *std::min_element(
      enemys_with_angle_diff.begin(), enemys_with_angle_diff.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });
    using boost::math::constants::degree;
    // 0~5°で0~1を遷移
    score *= std::clamp(best_intercepter.first * degree<double>() / 5.0, 0.0, 1.0);
  }

  if (world_model->point_checker.isPenaltyArea(p)) {
    score = 0.0;
  }
  return score;
}

auto PassTargetSelector::visualize(
  const WorldModelWrapper::SharedPtr & world_model, const Point & pass_origin, uint8_t target_id,
  const VisualizerMessageBuilder::SharedPtr & pass_builder) -> void
{
  auto receiver = world_model->getOurRobot(target_id);
  if (world_model->point_checker.isFieldInside(pass_origin)) {
    pass_builder->line()
      .start(pass_origin)
      .end(receiver->pose.pos)
      .stroke("lime", 0.8)
      .strokeWidth(40)
      .build();
    pass_builder->circle()
      .center(pass_origin)
      .radius(0.12)
      .stroke("lime")
      .strokeWidth(10)
      .fill("none")
      .build();
  }
  pass_builder->circle()
    .center(receiver->pose.pos)
    .radius(0.5)
    .stroke("lime")
    .strokeWidth(18)
    .fill("lime", 0.15)
    .build();
  pass_builder->text()
    .position(receiver->pose.pos.x(), receiver->pose.pos.y() + 0.35)
    .text(std::string("PASS TARGET #") + std::to_string(receiver->id))
    .fontSize(110)
    .fill("lime")
    .textAnchor("middle")
    .build();
}

auto PassTargetSelector::update(
  const WorldModelWrapper::SharedPtr & world_model,
  const std::deque<crane_msgs::msg::BallInfo> & ball_history,
  const VisualizerMessageBuilder::SharedPtr & pass_builder,
  crane_msgs::msg::GameAnalysis & analysis_msg) -> void
{
  // パス起点の決定
  const Point pass_origin = computePassOrigin(world_model, ball_history, analysis_msg);

  // 候補のスコア算出
  auto our_robots = world_model->ours().getAvailableRobots(true);
  std::vector<std::pair<std::shared_ptr<RobotInfo>, double>> score_with_bots;
  score_with_bots.reserve(our_robots.size());
  for (const auto & robot : our_robots) {
    if (robot->id == world_model->getOurGoalieId()) {
      continue;  // GK除外
    }
    if (robot->pose.pos.x() * world_model->getOurSideSign() > 0.0) {
      continue;  // 自陣除外
    }
    if (world_model->point_checker.isPenaltyArea(robot->pose.pos)) {
      continue;  // PA内除外
    }
    double score = calcScore(world_model, pass_origin, robot->pose.pos);
    score_with_bots.emplace_back(robot, score);
  }

  std::sort(score_with_bots.begin(), score_with_bots.end(), [](const auto & a, const auto & b) {
    return a.second > b.second;
  });

  analysis_msg.pass_scores.clear();
  analysis_msg.pass_scores.reserve(score_with_bots.size());
  for (const auto & [robot, score] : score_with_bots) {
    crane_msgs::msg::FloatWithID msg;
    msg.set__id(robot->id).set__value(score);
    analysis_msg.pass_scores.push_back(msg);
  }

  // ヒステリシスによるターゲット選定
  analysis_msg.pass_target_id = -1;
  if (!analysis_msg.pass_scores.empty()) {
    const auto & best = analysis_msg.pass_scores.front();
    const int best_id = static_cast<int>(best.id);
    const double best_score = static_cast<double>(best.value);

    const rclcpp::Time now_time = ros_clock_.now();
    const bool hold_active = (now_time - last_switch_time_).seconds() < min_hold_duration_sec_;

    double prev_score = -1.0;
    if (last_pass_target_id_.has_value()) {
      auto it = std::find_if(
        analysis_msg.pass_scores.begin(), analysis_msg.pass_scores.end(),
        [&](const crane_msgs::msg::FloatWithID & s) {
          return s.id == last_pass_target_id_.value();
        });
      if (it != analysis_msg.pass_scores.end()) prev_score = it->value;
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

    analysis_msg.pass_target_id = last_pass_target_id_.value();

    // 可視化
    if (pass_builder) {
      visualize(
        world_model, pass_origin, static_cast<uint8_t>(analysis_msg.pass_target_id), pass_builder);
    }
  }
}

}  // namespace crane
