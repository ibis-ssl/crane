// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/world_model_wrapper.hpp"

namespace crane
{
void BallContact::update(bool is_contacted)
{
  auto now = std::chrono::system_clock::now();
  if (is_contacted) {
    last_contact_end_time = now;
    if (not is_contacted_pre_frame) {
      last_contact_start_time = now;
    }
  } else {
    last_contact_start_time = last_contact_end_time;
  }
  is_contacted_pre_frame = is_contacted;
}

void Hysteresis::update(double value)
{
  if (not is_high && value > upper_threshold) {
    is_high = true;
    upper_callback();
  }

  if (is_high && value < lower_threshold) {
    is_high = false;
    lower_callback();
  }
}

auto Ball::isMovingTowards(const Point & p, double angle_threshold_deg, double near_threshold) const
  -> bool
{
  if ((pos - p).norm() < near_threshold) {
    return false;
  } else {
    auto dir = (p - pos).normalized();
    return dir.dot(vel.normalized()) > cos(angle_threshold_deg * M_PI / 180.0);
  }
}

WorldModelWrapper::WorldModelWrapper(rclcpp::Node & node)
: ball_owner_calculator(this), point_checker(this)
{
  // メモリ確保
  // ヒトサッカーの台数は超えないはず
  constexpr uint8_t MAX_ROBOT_NUM = 20;
  for (int i = 0; i < MAX_ROBOT_NUM; i++) {
    ours.robots.emplace_back(std::make_shared<RobotInfo>());
    theirs.robots.emplace_back(std::make_shared<RobotInfo>());
  }

  subscriber = node.create_subscription<crane_msgs::msg::WorldModel>(
    "/world_model", 10, [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void {
      latest_msg = *msg;
      this->update(*msg);
      has_updated = true;
      for (auto & callback : callbacks) {
        callback();
      }
    });
}

void WorldModelWrapper::update(const crane_msgs::msg::WorldModel & world_model)
{
  play_situation.update(world_model.play_situation);

  for (auto & our_robot : ours.robots) {
    our_robot->available = false;
  }

  for (auto & their_robot : theirs.robots) {
    their_robot->available = false;
  }

  ball.pos << world_model.ball_info.pose.x, world_model.ball_info.pose.y;
  ball.vel << world_model.ball_info.velocity.x, world_model.ball_info.velocity.y;
  ball.ball_speed_hysteresis.update(ball.vel.norm());

  for (auto & robot : world_model.robot_info_ours) {
    auto & info = ours.robots.at(robot.id);
    info->available = !robot.disappeared;
    if (info->available) {
      info->id = robot.id;
      info->detection_stamp = robot.detection_stamp;
      info->pose.pos << robot.pose.x, robot.pose.y;
      info->pose.theta = robot.pose.theta;
      info->vel.linear << robot.velocity.x, robot.velocity.y;
      info->ball_contact.update((info->kicker_center() - ball.pos).norm() < 0.1);
    } else {
      info->ball_contact.update(false);
    }
  }

  for (auto robot : world_model.robot_info_theirs) {
    auto & info = theirs.robots.at(robot.id);
    info->available = !robot.disappeared;
    if (info->available) {
      info->id = robot.id;
      info->ball_contact.update(
        robot.ball_contact.current_time == robot.ball_contact.last_contacted_time);
      info->pose.pos << robot.pose.x, robot.pose.y;
      info->pose.theta = robot.pose.theta;
      info->vel.linear << robot.velocity.x, robot.velocity.y;
      // todo : omega
    } else {
      info->ball_contact.update(false);
    }
  }

  field_size << world_model.field_info.x, world_model.field_info.y;
  penalty_area_size << world_model.penalty_area_size.x, world_model.penalty_area_size.y;

  goal_size << world_model.goal_size.x, world_model.goal_size.y;
  goal << getOurSideSign() * field_size.x() * 0.5, 0.;

  if (onPositiveHalf()) {
    ours.penalty_area.max_corner() << goal.x(), goal.y() + world_model.penalty_area_size.y / 2.;
    ours.penalty_area.min_corner() << goal.x() - world_model.penalty_area_size.x,
      goal.y() - world_model.penalty_area_size.y / 2.;
  } else {
    ours.penalty_area.max_corner() << goal.x() + world_model.penalty_area_size.x,
      goal.y() + world_model.penalty_area_size.y / 2.;
    ours.penalty_area.min_corner() << goal.x(), goal.y() - world_model.penalty_area_size.y / 2.;
  }
  theirs.penalty_area.max_corner()
    << std::max(-ours.penalty_area.max_corner().x(), -ours.penalty_area.min_corner().x()),
    ours.penalty_area.max_corner().y();
  theirs.penalty_area.min_corner()
    << std::min(-ours.penalty_area.max_corner().x(), -ours.penalty_area.min_corner().x()),
    ours.penalty_area.min_corner().y();

  if (ball_owner_calculator_enabled) {
    ball_owner_calculator.update();
  }
}

auto WorldModelWrapper::generateFieldPoints(float grid_size) const
{
  std::vector<Point> points;
  for (float x = 0.f; x <= field_size.x() / 2.f; x += grid_size) {
    for (float y = 0.f; y <= field_size.y() / 2.f; y += grid_size) {
      points.emplace_back(x, y);
    }
  }
  return points;
}

auto WorldModelWrapper::getNearestRobotWithDistanceFromPoint(
  const Point & point, const std::vector<std::shared_ptr<RobotInfo>> robots) const
  -> std::pair<std::shared_ptr<RobotInfo>, double>
{
  if (robots.empty()) {
    throw std::runtime_error("getNearestRobotWithDistanceFromPoint: robots is empty");
  }
  auto nearest_robot =
    std::ranges::min_element(robots, [&point](const auto & robot1, const auto & robot2) {
      return (robot1->pose.pos - point).squaredNorm() < (robot2->pose.pos - point).squaredNorm();
    });
  double min_sq_distance = (nearest_robot->pose.pos - point).squaredNorm();
  return {*nearest_robot, std::sqrt(min_sq_distance)};
}

auto WorldModelWrapper::getNearestRobotWithDistanceFromSegment(
  const Segment & segment, const std::vector<std::shared_ptr<RobotInfo>> robots) const
  -> std::pair<std::shared_ptr<RobotInfo>, double>
{
  if (robots.empty()) {
    throw std::runtime_error("getNearestRobotWithDistanceFromSegment: robots is empty");
  }
  auto nearest_robot =
    std::ranges::min_element(robots, [&segment](const auto & robot1, const auto & robot2) {
      return bg::distance(segment, robot1->pose.pos) < bg::distance(segment, robot2->pose.pos);
    });
  double min_distance = bg::distance(segment, nearest_robot->pose.pos);
  return {*nearest_robot, min_distance};
}

auto WorldModelWrapper::PointChecker::isFieldInside(const Point & p, double offset) const -> bool
{
  Box field_box;
  field_box.min_corner() << -world_model->field_size.x() / 2.f - offset,
    -world_model->field_size.y() / 2.f - offset;
  field_box.max_corner() << world_model->field_size.x() / 2.f + offset,
    world_model->field_size.y() / 2.f + offset;
  return isInBox(field_box, p);
}

auto WorldModelWrapper::PointChecker::isBallPlacementArea(const Point & p, double offset) const
  -> bool
{
  // During ball placement, all robots of the non-placing team have to keep
  // at least 0.5 meters distance to the line between the ball and the placement position
  // (the forbidden area forms a stadium shape).
  // ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement_interference
  //    Segment ball_placement_line;
  //    {Point(ball_placement_target), Point(ball.pos)};
  if (auto area = world_model->getBallPlacementArea(offset)) {
    return bg::distance(area.value(), p) < 0.001;
  } else {
    return false;
  }
}

auto WorldModelWrapper::PointChecker::isEnemyPenaltyArea(const Point & p, double offset) const
  -> bool
{
  return isInBox(world_model->theirs.penalty_area, p, offset);
}

auto WorldModelWrapper::PointChecker::isFriendPenaltyArea(const Point & p, double offset) const
  -> bool
{
  return isInBox(world_model->ours.penalty_area, p, offset);
}

auto WorldModelWrapper::PointChecker::isPenaltyArea(const Point & p, double offset) const -> bool
{
  return isFriendPenaltyArea(p, offset) || isEnemyPenaltyArea(p, offset);
}

auto WorldModelWrapper::getBallPlacementTarget() const -> std::optional<Point>
{
  if (
    play_situation.getSituationCommandID() == crane_msgs::msg::PlaySituation::OUR_BALL_PLACEMENT or
    play_situation.getSituationCommandID() ==
      crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT) {
    return play_situation.placement_position;
  } else {
    return std::nullopt;
  }
}

auto WorldModelWrapper::getBallPlacementArea(const double offset) const -> std::optional<Capsule>
{
  if (auto target = getBallPlacementTarget()) {
    Capsule area;
    area.segment.first = ball.pos;
    area.segment.second = target.value();
    area.radius = 0.5 + offset;
    return area;
  } else {
    return std::nullopt;
  }
}

auto WorldModelWrapper::getLargestGoalAngleRangeFromPoint(Point from) -> std::pair<double, double>
{
  Interval goal_range;

  auto goal_posts = getTheirGoalPosts();
  if (goal_posts.first.x() < 0.) {
    goal_range.append(
      normalizeAngle(getAngle(goal_posts.first - from) + M_PI),
      normalizeAngle(getAngle(goal_posts.second - from) + M_PI));
  } else {
    goal_range.append(getAngle(goal_posts.first - from), getAngle(goal_posts.second - from));
  }

  for (auto & enemy : theirs.getAvailableRobots()) {
    double distance = enemy->getDistance(from);
    constexpr double MACHINE_RADIUS = 0.1;

    double center_angle = [&]() {
      if (goal_posts.first.x() < 0.) {
        return normalizeAngle(getAngle(enemy->pose.pos - from) + M_PI);
      } else {
        return getAngle(enemy->pose.pos - from);
      }
    }();
    double diff_angle =
      atan(MACHINE_RADIUS / std::sqrt(distance * distance - MACHINE_RADIUS * MACHINE_RADIUS));

    goal_range.erase(center_angle - diff_angle, center_angle + diff_angle);
  }

  auto largest_interval = goal_range.getLargestInterval();

  double target_angle = [&]() {
    if (goal_posts.first.x() < 0.) {
      return normalizeAngle((largest_interval.first + largest_interval.second) / 2.0 - M_PI);
    } else {
      return (largest_interval.first + largest_interval.second) / 2.0;
    }
  }();

  return {target_angle, largest_interval.second - largest_interval.first};
}

auto WorldModelWrapper::getLargestOurGoalAngleRangeFromPoint(
  Point from, std::vector<std::shared_ptr<RobotInfo>> robots) -> std::pair<double, double>
{
  if (robots.empty()) {
    throw std::runtime_error("getLargestOurGoalAngleRangeFromPoint: robots is empty");
  }
  Interval goal_range;

  auto goal_posts = getOurGoalPosts();
  if (goal_posts.first.x() < 0.) {
    goal_range.append(
      normalizeAngle(getAngle(goal_posts.first - from) + M_PI),
      normalizeAngle(getAngle(goal_posts.second - from) + M_PI));
  } else {
    goal_range.append(getAngle(goal_posts.first - from), getAngle(goal_posts.second - from));
  }

  for (auto & enemy : robots) {
    double distance = enemy->getDistance(from);
    constexpr double MACHINE_RADIUS = 0.1;

    double center_angle = [&]() {
      if (goal_posts.first.x() < 0.) {
        return normalizeAngle(getAngle(enemy->pose.pos - from) + M_PI);
      } else {
        return getAngle(enemy->pose.pos - from);
      }
    }();
    double diff_angle =
      atan(MACHINE_RADIUS / std::sqrt(distance * distance - MACHINE_RADIUS * MACHINE_RADIUS));

    goal_range.erase(center_angle - diff_angle, center_angle + diff_angle);
  }

  auto largest_interval = goal_range.getLargestInterval();

  double target_angle = [&]() {
    if (goal_posts.first.x() < 0.) {
      return normalizeAngle((largest_interval.first + largest_interval.second) / 2.0 - M_PI);
    } else {
      return (largest_interval.first + largest_interval.second) / 2.0;
    }
  }();

  return {target_angle, largest_interval.second - largest_interval.first};
}

auto WorldModelWrapper::getMinMaxSlackInterceptPointAndSlackTime(
  std::vector<std::shared_ptr<RobotInfo>> robots, double t_horizon, double t_step,
  double slack_time_offset)
  -> std::pair<std::optional<std::pair<Point, double>>, std::optional<std::pair<Point, double>>>
{
  auto ball_sequence = getBallSequence(t_horizon, t_step, ball.pos, ball.vel);
  std::optional<std::pair<Point, double>> max_intercept_point_and_time = std::nullopt;
  std::optional<std::pair<Point, double>> min_intercept_point_and_time = std::nullopt;
  double max_slack_time = -100.0;
  double min_slack_time = 100.0;
  for (const auto & [p_ball, t_ball] : ball_sequence) {
    if (not point_checker.isFieldInside(p_ball)) {
      // フィールド外は対象外
      continue;
    }

    if (const auto slack = getBallSlackTime(t_ball, robots); slack.has_value()) {
      auto slack_time = slack.value().slack_time + slack_time_offset;
      auto intercept_point = slack.value().intercept_point;
      if (slack_time > max_slack_time && slack_time > 0.) {
        max_slack_time = slack_time;
        max_intercept_point_and_time =
          std::make_optional<std::pair<Point, double>>({intercept_point, slack_time});
      }
      if (slack_time < min_slack_time && slack_time > 0.) {
        min_slack_time = slack_time;
        min_intercept_point_and_time =
          std::make_optional<std::pair<Point, double>>({intercept_point, slack_time});
      }
      //        if (visualizer) {
      //          visualizer->addPoint(p_ball, std::max(0., slack_time * 10), "red");
      //        }
    }
  }
  return {min_intercept_point_and_time, max_intercept_point_and_time};
}

auto WorldModelWrapper::getMinMaxSlackInterceptPoint(
  std::vector<std::shared_ptr<RobotInfo>> robots, double t_horizon, double t_step,
  double slack_time_offset) -> std::pair<std::optional<Point>, std::optional<Point>>
{
  auto [min_slack, max_slack] =
    getMinMaxSlackInterceptPointAndSlackTime(robots, t_horizon, t_step, slack_time_offset);
  std::optional<Point> min_intercept_point = std::nullopt;
  std::optional<Point> max_intercept_point = std::nullopt;
  if (min_slack.has_value()) {
    min_intercept_point = min_slack.value().first;
  }
  if (max_slack.has_value()) {
    max_intercept_point = max_slack.value().first;
  }
  return {min_intercept_point, max_intercept_point};
}

auto WorldModelWrapper::getBallSlackTime(
  double time, const std::vector<std::shared_ptr<RobotInfo>> & robots)
  -> std::optional<SlackTimeResult>
{
  // https://www.youtube.com/live/bizGFvaVUIk?si=mFZqirdbKDZDttIA&t=1452
  auto p_ball = getFutureBallPosition(ball.pos, ball.vel, time);
  if (p_ball) {
    Point intercept_point = p_ball.value() + ball.vel.normalized() * 0.3;
    double min_robot_time = std::numeric_limits<double>::max();
    std::shared_ptr<RobotInfo> best_robot = nullptr;
    for (const auto & robot : robots) {
      double t_robot = getTravelTimeTrapezoidal(robot, intercept_point);
      if (t_robot < min_robot_time) {
        min_robot_time = t_robot;
        best_robot = robot;
      }
    }
    if (min_robot_time != std::numeric_limits<double>::max()) {
      return std::make_optional<SlackTimeResult>(
        {time - min_robot_time, intercept_point, best_robot});
    } else {
      return std::nullopt;
    }
  } else {
    return std::nullopt;
  }
}

auto WorldModelWrapper::BallOwnerCalculator::update() -> void
{
  updateScore(true);
  updateScore(false);

  bool is_our_ball_old = std::exchange(is_our_ball, [&]() {
    if (not sorted_their_robots.empty() && not sorted_our_robots.empty()) {
      return sorted_our_robots.front().score > sorted_their_robots.front().score;
    } else {
      return is_our_ball;
    }
  }());

  uint8_t our_frontier_old = std::exchange(our_frontier, [&]() {
    if (not sorted_our_robots.empty()) {
      return sorted_our_robots.front().robot->id;
    } else {
      return our_frontier;
    }
  }());

  is_ball_owner_team_changed = is_our_ball_old != is_our_ball;
  if (is_ball_owner_team_changed) {
    std::cout << "ボールオーナーが" << (is_our_ball ? "我々" : "相手") << "チームに変更されました"
              << std::endl;
    if (ball_owner_team_change_callback) {
      ball_owner_team_change_callback(is_our_ball);
    }
  }

  is_our_ball_owner_changed = our_frontier_old != our_frontier;
  if (is_our_ball_owner_changed) {
    std::cout << "我々のボールオーナーが" << static_cast<int>(our_frontier_old) << "番から"
              << static_cast<int>(our_frontier) << "番に交代しました" << std::endl;
    if (ball_owner_id_change_callback) {
      ball_owner_id_change_callback(our_frontier);
    }
  }
}

auto WorldModelWrapper::BallOwnerCalculator::updateScore(bool our_team) -> void
{
  auto robots = our_team ? world_model->ours.getAvailableRobots(world_model->getOurGoalieId())
                         : world_model->theirs.getAvailableRobots();
  std::vector<RobotWithScore> & previous_sorted_robots(
    our_team ? sorted_our_robots : sorted_their_robots);

  std::vector<RobotWithScore> scores(robots.size());
  std::transform(
    robots.begin(), robots.end(), scores.begin(),
    [&](const std::shared_ptr<RobotInfo> & robot) { return calculateScore(robot); });

  // 前回の結果があればヒステリシスを加算
  if (not previous_sorted_robots.empty()) {
    if (auto previous_best_bot = std::find_if(
          scores.begin(), scores.end(),
          [&](const auto & e) { return e.robot->id == previous_sorted_robots.front().robot->id; });
        previous_best_bot != scores.end()) {
      // Slackタイム1.0秒のアドバンテージ
      previous_best_bot->score += 1.0;
    }
  }

  // スコアの高い順にソート
  std::sort(scores.begin(), scores.end(), [](const RobotWithScore & a, const RobotWithScore & b) {
    return a.score > b.score;
  });

  previous_sorted_robots = std::move(scores);
}

auto WorldModelWrapper::BallOwnerCalculator::calculateScore(
  const std::shared_ptr<RobotInfo> & robot) const
  -> WorldModelWrapper::BallOwnerCalculator::RobotWithScore
{
  RobotWithScore score;
  score.robot = robot;
  auto [min_slack, max_slack] =
    world_model->getMinMaxSlackInterceptPointAndSlackTime({robot}, 3.0, 0.1);
  if (min_slack.has_value()) {
    score.min_slack = min_slack->second;
  } else {
    score.min_slack = 100.;
  }
  if (max_slack.has_value()) {
    score.max_slack = max_slack->second;
  } else {
    score.max_slack = -100.;
  }
  // 3秒後にボールにたどり着けないロボットはスコアマイナス
  score.score = score.max_slack + 3.0;
  return score;
}
}  // namespace crane
