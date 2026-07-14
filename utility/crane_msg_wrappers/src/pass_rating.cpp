// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/pass_rating.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/pass_evaluation.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/range/primitives.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{
auto ratePassCandidate(
  WorldModelWrapper * world_model, const Point & pass_origin, const Point & target,
  const PassRatingConfig & config) -> PassRating
{
  auto * wm = world_model;

  const double pass_distance = (target - pass_origin).norm();

  // 敵ゴール見通し / 自ゴール危険度の角度幅
  const double their_goal_angle_width = wm->getLargestGoalAngleRangeFromPoint(target).angle_width;
  const double own_goal_angle_width =
    wm->getLargestGoalAngleRangeFromPoint(target, wm->getOurGoalPosts(), {}).angle_width;

  // 敵ゴールへの正規化距離
  const double normed_distance_to_their_goal =
    ((target - wm->getTheirGoalCenter()).norm() - (wm->fieldSize().x() * 0.5)) /
    (wm->fieldSize().x() * 0.5);

  // 敵インターセプト評価（キック速度・ball_velocity は PassTargetMetric と同式）
  const double kick_speed = std::clamp(pass_distance, 2.0, 4.0);
  const Segment pass_line{pass_origin, target};
  const Vector2 pass_dir = target - pass_origin;
  const Vector2 ball_velocity =
    (pass_distance > 1e-6) ? Vector2(pass_dir / pass_distance * kick_speed) : Vector2::Zero();

  auto calc_slack_time = [&](const auto & enemy) -> double {
    const auto closest = getClosestPointAndDistance(enemy->pose.pos, pass_line);
    const double ball_time = (closest.closest_point - pass_origin).norm() / kick_speed;

    auto slack_result =
      wm->getBallSlackTime(pass_origin, ball_velocity, ball_time, {enemy}, config.enemy_slack);

    // getBallSlackTime の slack_time = ball_time - robot_time（正なら渡したロボットが
    // ボールに先着＝インターセプト可能）。ここでは敵を渡すため符号を反転し、
    // 「敵がボールに間に合わない安全余裕（robot_time - ball_time）」として扱う。
    return slack_result.has_value() ? -slack_result->slack_time : 1.0;
  };

  // 遮蔽(shadow)は全敵、slack はフィルタ済み敵。2つのリストを潰さないこと（挙動保存）。
  auto enemies = wm->theirs().robotsWhere().available().get();
  auto slack_times_view = enemies | ranges::views::filter([&](const auto & enemy) {
                            // パス起点から近すぎる敵はチップで飛び越せるので除外
                            return enemy->getDistance(pass_origin) >= 1.0;
                          }) |
                          ranges::views::filter([&](const auto & enemy) {
                            // パス先より向こうにいる敵は除外
                            return pass_dir.dot(enemy->pose.pos - target) <= 0.0;
                          }) |
                          ranges::views::transform(calc_slack_time);
  const double worst_slack = ranges::empty(slack_times_view) ? 1.0 : ranges::min(slack_times_view);

  const double shadow_score = evaluatePassShadow(pass_origin, target, enemies);

  PassScoreTerms terms;
  terms.pass_distance = pass_distance;
  terms.their_goal_angle_width = their_goal_angle_width;
  terms.own_goal_angle_width = own_goal_angle_width;
  terms.normed_distance_to_their_goal = normed_distance_to_their_goal;
  terms.worst_enemy_slack = worst_slack;
  terms.slack_scale = config.slack_scale;
  terms.shadow_score = shadow_score;
  terms.in_penalty_area = wm->point_checker.isPenaltyArea(target);

  return combinePassScore(terms);
}
}  // namespace crane
