// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/threat_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_physics/travel_time.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

namespace crane
{

ThreatEvaluator::ThreatEvaluator(const ThreatEvaluatorConfig & config) : config_(config) {}

auto ThreatEvaluator::calculateBallThreat(const WorldModelWrapper & world_model) -> BallThreat
{
  BallThreat threat;

  auto [source_type, source_pos] = determineBallThreatSource(world_model);
  threat.source_type = source_type;
  threat.source_position = source_pos;
  threat.velocity = world_model.ball().vel;

  // 脅威ライン: 脅威元からゴール中央へ
  Point goal_center = world_model.getOurGoalCenter();
  threat.threat_line = Segment{source_pos, goal_center};

  // 防御ライン計算
  threat.protection_line = calculateProtectionLine(threat.threat_line, 0.3, world_model);

  return threat;
}

auto ThreatEvaluator::calculateRobotThreats(
  const WorldModelWrapper & world_model, const BallThreat & ball_threat) -> std::vector<RobotThreat>
{
  std::vector<RobotThreat> threats;

  auto enemy_robots = world_model.theirs().robotsWhere().available().get();
  Point ball_pos = ball_threat.source_position;
  Point goal_center = world_model.getOurGoalCenter();

  for (const auto & robot : enemy_robots) {
    RobotThreat threat;
    threat.robot = robot;  // RobotInfoポインタを保持

    // 脅威ライン
    threat.threat_line = Segment{robot->pose.pos, goal_center};

    // 脅威評価（RobotInfoを直接使用）
    threat.rating_detail = rateRobotThreat(ball_pos, robot, world_model);
    threat.threat_rating = threat.rating_detail.total_score;

    // 防御ライン計算
    threat.protection_line = calculateProtectionLine(threat.threat_line, 0.3, world_model);

    // 守備戦略推奨（ボールアクセススコアで判定）
    threat.recommended_strategy = (threat.rating_detail.score_ball_access > 0.5)
                                    ? RobotThreat::DefenseStrategy::MAN_TO_MAN
                                    : RobotThreat::DefenseStrategy::CENTER_BACK;

    threats.push_back(threat);
  }

  // 脅威度でソート（降順）
  ranges::sort(
    threats, [](const auto & a, const auto & b) { return a.threat_rating > b.threat_rating; });

  return threats;
}

auto ThreatEvaluator::rateRobotThreat(
  const Point & ball_pos, const std::shared_ptr<RobotInfo> & robot,
  const WorldModelWrapper & world_model) -> ThreatRatingDetail
{
  ThreatRatingDetail detail;
  Point threat_pos = robot->pose.pos;

  // 4因子のスコア計算
  detail.score_redirect_angle = calcRedirectAngleScore(ball_pos, robot, world_model);
  detail.score_pen_area_border = calcPenAreaBorderScore(threat_pos, world_model);
  detail.score_facing_goal = calcFacingGoalScore(robot, world_model);
  detail.score_ball_access = calcBallAccessScore(ball_pos, robot);

  // 距離係数
  detail.distance_factor =
    calcDistanceToGoalFactor(threat_pos, config_.danger_dropoff_x, world_model);

  // 重み付け合計（4因子）
  double weighted_sum = config_.weight_redirect_angle * detail.score_redirect_angle +
                        config_.weight_pen_area_border * detail.score_pen_area_border +
                        config_.weight_facing_goal * detail.score_facing_goal +
                        config_.weight_ball_access * detail.score_ball_access;

  double weight_sum = config_.weight_redirect_angle + config_.weight_pen_area_border +
                      config_.weight_facing_goal + config_.weight_ball_access;

  detail.total_score = (weighted_sum / weight_sum) * detail.distance_factor;
  detail.total_score = std::clamp(detail.total_score, 0.0, 1.0);

  return detail;
}

auto ThreatEvaluator::calculateRecommendedDefenders(
  [[maybe_unused]] const BallThreat & ball_threat, const std::vector<RobotThreat> & robot_threats,
  int available_robots) -> int
{
  // 基本: 脅威数に応じて守備者を割り当て
  // 最低1人、最大で利用可能ロボット数の半分

  // 高脅威ロボットの数をカウント
  int high_threat_count = 0;
  for (const auto & threat : robot_threats) {
    if (threat.threat_rating > 0.5) {
      high_threat_count++;
    }
  }

  // 推奨守備者数: 高脅威数 + 1（ボール対応）
  int recommended = high_threat_count + 1;

  // 範囲制限
  int max_defenders = std::max(1, available_robots / 2);
  return std::clamp(recommended, 1, max_defenders);
}

auto ThreatEvaluator::toThreatInfoMsg(const BallThreat & threat) const
  -> crane_msgs::msg::ThreatInfo
{
  crane_msgs::msg::ThreatInfo msg;

  msg.threat_type = crane_msgs::msg::ThreatInfo::THREAT_TYPE_BALL;
  msg.source_position.x = threat.source_position.x();
  msg.source_position.y = threat.source_position.y();
  msg.source_position.z = 0.0;

  msg.threat_line_start.x = threat.threat_line.first.x();
  msg.threat_line_start.y = threat.threat_line.first.y();
  msg.threat_line_end.x = threat.threat_line.second.x();
  msg.threat_line_end.y = threat.threat_line.second.y();

  if (threat.protection_line) {
    msg.has_protection_line = true;
    msg.protection_line_start.x = threat.protection_line->first.x();
    msg.protection_line_start.y = threat.protection_line->first.y();
    msg.protection_line_end.x = threat.protection_line->second.x();
    msg.protection_line_end.y = threat.protection_line->second.y();
  } else {
    msg.has_protection_line = false;
  }

  msg.velocity.x = threat.velocity.x();
  msg.velocity.y = threat.velocity.y();

  // ソースタイプ
  switch (threat.source_type) {
    case BallThreat::SourceType::BALL:
      msg.source_type = crane_msgs::msg::ThreatInfo::SOURCE_TYPE_BALL;
      break;
    case BallThreat::SourceType::GOAL_SHOT:
      msg.source_type = crane_msgs::msg::ThreatInfo::SOURCE_TYPE_GOAL_SHOT;
      break;
    case BallThreat::SourceType::PASS_RECEIVE:
      msg.source_type = crane_msgs::msg::ThreatInfo::SOURCE_TYPE_PASS_RECEIVE;
      break;
    case BallThreat::SourceType::BOT_CLOSE_TO_BALL:
      msg.source_type = crane_msgs::msg::ThreatInfo::SOURCE_TYPE_BOT_CLOSE_TO_BALL;
      break;
  }

  return msg;
}

auto ThreatEvaluator::toThreatInfoMsg(const RobotThreat & threat) const
  -> crane_msgs::msg::ThreatInfo
{
  crane_msgs::msg::ThreatInfo msg;

  msg.threat_type = crane_msgs::msg::ThreatInfo::THREAT_TYPE_ROBOT;
  msg.source_robot_id = threat.robot->id;  // ポインタ経由でアクセス
  msg.source_position.x = threat.robot->pose.pos.x();
  msg.source_position.y = threat.robot->pose.pos.y();
  msg.source_position.z = 0.0;

  msg.threat_line_start.x = threat.threat_line.first.x();
  msg.threat_line_start.y = threat.threat_line.first.y();
  msg.threat_line_end.x = threat.threat_line.second.x();
  msg.threat_line_end.y = threat.threat_line.second.y();

  if (threat.protection_line) {
    msg.has_protection_line = true;
    msg.protection_line_start.x = threat.protection_line->first.x();
    msg.protection_line_start.y = threat.protection_line->first.y();
    msg.protection_line_end.x = threat.protection_line->second.x();
    msg.protection_line_end.y = threat.protection_line->second.y();
  } else {
    msg.has_protection_line = false;
  }

  msg.velocity.x = threat.robot->vel.linear.x();
  msg.velocity.y = threat.robot->vel.linear.y();

  msg.threat_rating = static_cast<float>(threat.threat_rating);
  msg.score_redirect_angle = static_cast<float>(threat.rating_detail.score_redirect_angle);
  msg.score_pen_area_border = static_cast<float>(threat.rating_detail.score_pen_area_border);
  msg.score_facing_goal = static_cast<float>(threat.rating_detail.score_facing_goal);
  msg.score_ball_access = static_cast<float>(threat.rating_detail.score_ball_access);

  return msg;
}

// ===== プライベート関数（4因子） =====

auto ThreatEvaluator::calcRedirectAngleScore(
  const Point & ball_pos, const std::shared_ptr<RobotInfo> & robot,
  const WorldModelWrapper & wm) const -> double
{
  // キッカー位置を使用
  Point kicker_pos = robot->kicker_center();
  Vector2 ball_to_kicker = (kicker_pos - ball_pos).normalized();

  // ゴール方向
  Point goal_center = wm.getOurGoalCenter();
  Vector2 kicker_to_goal = (goal_center - kicker_pos).normalized();

  // リダイレクト角度（ボール→キッカー→ゴール）
  double redirect_angle = std::acos(std::clamp(-ball_to_kicker.dot(kicker_to_goal), -1.0, 1.0));

  // ロボットの向きも考慮（キックできる方向に向いているか）
  Vector2 robot_facing = getNormVec(robot->pose.theta);
  double facing_factor = (robot_facing.dot(kicker_to_goal) > 0) ? 1.0 : 0.5;  // 逆向きは脅威半減

  // 0-45度: 高脅威, 45-90度: 中脅威, 90度以上: 低脅威
  double angle_score = 1.0 - std::min(redirect_angle / (M_PI / 2.0), 1.0);

  return std::clamp(angle_score * facing_factor, 0.0, 1.0);
}

auto ThreatEvaluator::calcFacingGoalScore(
  const std::shared_ptr<RobotInfo> & robot, const WorldModelWrapper & wm) const -> double
{
  Point goal_center = wm.getOurGoalCenter();
  Vector2 robot_to_goal = (goal_center - robot->pose.pos).normalized();
  Vector2 robot_facing = getNormVec(robot->pose.theta);

  // 内積: 1.0 = 完全にゴール方向, -1.0 = 逆方向
  double dot = robot_to_goal.dot(robot_facing);
  // -1〜1を0〜1にマップ
  return std::clamp((dot + 1.0) / 2.0, 0.0, 1.0);
}

auto ThreatEvaluator::calcBallAccessScore(
  const Point & ball_pos, const std::shared_ptr<RobotInfo> & robot) const -> double
{
  // 台形速度プロファイルで到達時間を計算
  double travel_time =
    getTravelTimeTrapezoidal(robot->pose.pos, robot->vel.linear, ball_pos, 3.0, 2.0);

  // 2秒以内でアクセス可能なら高スコア（1秒以内で最大）
  return std::clamp(1.0 - travel_time / 2.0, 0.0, 1.0);
}

auto ThreatEvaluator::calcPenAreaBorderScore(
  const Point & threat_pos, const WorldModelWrapper & wm) const -> double
{
  // ペナルティエリアに近いほど高スコア
  if (wm.point_checker.isFriendPenaltyArea(threat_pos)) {
    return 1.0;  // ペナルティエリア内は最高脅威
  }

  // ペナルティエリアまでの距離で評価
  Point goal_center = wm.getOurGoalCenter();
  double defense_height = wm.getDefenseHeight();
  double defense_width = wm.getDefenseWidth();

  // 簡易距離計算
  double dist_x = std::abs(std::abs(threat_pos.x()) - std::abs(goal_center.x())) - defense_height;
  double dist_y = std::max(0.0, std::abs(threat_pos.y()) - defense_width / 2.0);
  double dist = std::sqrt(dist_x * dist_x + dist_y * dist_y);

  // 3m以内で線形減衰
  double score = 1.0 - (dist / 3.0);
  return std::clamp(score, 0.0, 1.0);
}

auto ThreatEvaluator::calcDistanceToGoalFactor(
  const Point & threat_pos, double dropoff_percentage, const WorldModelWrapper & wm) const -> double
{
  Point goal_center = wm.getOurGoalCenter();
  double field_length = wm.fieldSize().x();

  double dist_to_goal = (threat_pos - goal_center).norm();

  // フィールド長の一定割合以降は減衰
  double dropoff_start = field_length * dropoff_percentage;

  if (dist_to_goal < dropoff_start) {
    return 1.0;
  }

  double factor = 1.0 - ((dist_to_goal - dropoff_start) / (field_length - dropoff_start));
  return std::clamp(factor, 0.0, 1.0);
}

auto ThreatEvaluator::determineBallThreatSource(const WorldModelWrapper & wm)
  -> std::pair<BallThreat::SourceType, Point>
{
  const auto & ball = wm.ball();

  // ゴールに向かって動いているか
  auto goal_posts = wm.getOurGoalPosts();
  Segment goal_line{goal_posts.first, goal_posts.second};
  Segment ball_trajectory = ball.getTrajectorySegmentByDistance(10.0);

  auto intersections = getIntersections(ball_trajectory, goal_line);
  if (!intersections.empty() && ball.vel.norm() > config_.check_ball_direction_vel_threshold) {
    return {BallThreat::SourceType::GOAL_SHOT, ball.pos};
  }

  // ボールに最も近い敵ロボット
  auto enemies = wm.theirs().robotsWhere().available().get();
  if (!enemies.empty()) {
    auto closest =
      std::min_element(enemies.begin(), enemies.end(), [&](const auto & a, const auto & b) {
        return a->getDistance(ball.pos) < b->getDistance(ball.pos);
      });

    if ((*closest)->getDistance(ball.pos) < 0.5) {
      return {BallThreat::SourceType::BOT_CLOSE_TO_BALL, (*closest)->pose.pos};
    }
  }

  // デフォルト: ボール位置
  return {BallThreat::SourceType::BALL, ball.pos};
}

auto ThreatEvaluator::calculateProtectionLine(
  const Segment & threat_line, double min_distance, const WorldModelWrapper & wm)
  -> std::optional<Segment>
{
  // 脅威ラインとペナルティエリアの交点を計算
  auto intersection = wm.getIntersectionOurPenaltyArea(threat_line, 0.0, 0.0);

  if (!intersection) {
    return std::nullopt;
  }

  // 交点からmin_distance離れた位置に防御ラインを設定
  Vector2 direction = (threat_line.second - threat_line.first).normalized();
  Point start = *intersection - direction * min_distance;

  // 防御ラインの幅（ゴール幅に基づく）
  Vector2 perpendicular{-direction.y(), direction.x()};
  double line_width = 0.5;  // 守備者1人分の幅

  return Segment{start - perpendicular * line_width, start + perpendicular * line_width};
}

}  // namespace crane
