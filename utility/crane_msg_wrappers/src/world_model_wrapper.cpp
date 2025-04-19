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

WorldModelWrapper::WorldModelWrapper(rclcpp::Node & node, bool setup_subscriber)
: ball_owner_calculator(this), point_checker(this)
{
  // メモリ確保
  // ヒトサッカーの台数は超えないはず
  constexpr uint8_t MAX_ROBOT_NUM = 20;
  for (int i = 0; i < MAX_ROBOT_NUM; i++) {
    ours.robots.emplace_back(std::make_shared<RobotInfo>());
    theirs.robots.emplace_back(std::make_shared<RobotInfo>());
  }

  if (setup_subscriber) {
    subscriber = node.create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10,
      [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void { this->update(*msg); });
  }
}

void WorldModelWrapper::update(const crane_msgs::msg::WorldModel & world_model)
{
  has_updated = true;
  latest_msg = world_model;
  for (auto & our_robot : ours.robots) {
    our_robot->available = false;
  }

  for (auto & their_robot : theirs.robots) {
    their_robot->available = false;
  }

  ours.max_allowed_bots = world_model.our_max_allowed_bots;
  theirs.max_allowed_bots = world_model.their_max_allowed_bots;

  ball.pos << world_model.ball_info.pose.x, world_model.ball_info.pose.y;
  ball.vel << world_model.ball_info.velocity.x, world_model.ball_info.velocity.y;
  ball.ball_speed_hysteresis.update(ball.vel.norm());
  ball.detected = world_model.ball_info.detected;

  for (auto & robot : world_model.robot_info_ours) {
    auto & info = ours.robots.at(robot.id);
    info->available = robot.detected;
    if (info->available) {
      info->id = robot.id;
      info->vision_detection_stamp = robot.last_vision_detection_stamp;
      info->pose.pos << robot.pose.x, robot.pose.y;
      info->pose.theta = robot.pose.theta;
      info->vel.linear << robot.velocity.x, robot.velocity.y;
      info->ball_contact.update((info->kicker_center() - ball.pos).norm() < 0.1);
      // ボールセンサは味方だけ
      info->ball_sensor = robot.ball_sensor;
      info->ball_sensor_stamp = robot.last_ball_sensor_stamp;
    } else {
      info->ball_contact.update(false);
    }
  }

  for (auto robot : world_model.robot_info_theirs) {
    auto & info = theirs.robots.at(robot.id);
    info->available = robot.detected;
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

  ours.goalie_id = world_model.our_goalie_id;
  theirs.goalie_id = world_model.their_goalie_id;

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

  for (auto & callback : callbacks) {
    callback();
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

auto WorldModelWrapper::getNearestRobotWithDistanceFromSegment(
  const Segment & segment, const RobotList & robots) const -> std::optional<RobotWithDistance>
{
  if (robots.empty()) {
    return std::nullopt;
  }
  auto nearest_robot = ranges::min(robots, [&segment](const auto & robot1, const auto & robot2) {
    return bg::distance(segment, robot1->pose.pos) < bg::distance(segment, robot2->pose.pos);
  });
  double min_distance = bg::distance(segment, nearest_robot->pose.pos);
  return std::make_optional<RobotWithDistance>(nearest_robot, min_distance);
}

auto WorldModelWrapper::getNearestRobotWithDistanceFromPoint(
  const Point & point, const RobotList & robots) const -> std::optional<RobotWithDistance>
{
  if (robots.empty()) {
    return std::nullopt;
  }
  auto nearest_robot = ranges::min(robots, [point](const auto & robot1, const auto & robot2) {
    return (robot1->pose.pos - point).norm() < (robot2->pose.pos - point).norm();
  });
  double min_distance = (nearest_robot->pose.pos - point).norm();
  return std::make_optional<RobotWithDistance>(nearest_robot, min_distance);
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
    latest_msg.play_situation.command.value == crane_msgs::msg::PlaySituation::OUR_BALL_PLACEMENT or
    latest_msg.play_situation.command.value ==
      crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT) {
    const auto designated_position = latest_msg.play_situation.referee_raw.designated_position;
    if (not designated_position.empty()) {
      return Point(designated_position.front().x / 1000., designated_position.front().y / 1000.);
    }
  }
  return std::nullopt;
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

auto WorldModelWrapper::getLargestGoalAngleRangeFromPoint(Point from) const -> GoalAngleRange
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
      return normalizeAngle(
        getIntermediateAngle(largest_interval.first, largest_interval.second) - M_PI);
    } else {
      return getIntermediateAngle(largest_interval.first, largest_interval.second);
    }
  }();

  return {target_angle, getAngleDiff(largest_interval.second, largest_interval.first)};
}

auto WorldModelWrapper::getLargestOurGoalAngleRangeFromPoint(
  Point from, const RobotList & robots) const -> GoalAngleRange
{
  Interval goal_range;

  auto goal_posts = getOurGoalPosts();
  if (goal_posts.first.x() < 0.) {
    goal_range.append(
      normalizeAngle(getAngle(goal_posts.first - from) + M_PI),
      normalizeAngle(getAngle(goal_posts.second - from) + M_PI));
  } else {
    goal_range.append(getAngle(goal_posts.first - from), getAngle(goal_posts.second - from));
  }

  if (ranges::empty(robots)) {
    ranges::for_each(robots, [&](const auto & enemy) {
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
    });
  }

  auto largest_interval = goal_range.getLargestInterval();

  double target_angle = [&]() {
    if (goal_posts.first.x() < 0.) {
      return normalizeAngle((largest_interval.first + largest_interval.second) / 2.0 - M_PI);
    } else {
      return (largest_interval.first + largest_interval.second) / 2.0;
    }
  }();

  GoalAngleRange range;
  range.center_angle = target_angle;
  range.angle_width = largest_interval.second - largest_interval.first;
  return range;
}

auto WorldModelWrapper::getBallSlackTime(
  double time, const RobotList & robots, const double max_acc, const double max_vel)
  -> std::optional<SlackTimeResult>
{
  // https://www.youtube.com/live/bizGFvaVUIk?si=mFZqirdbKDZDttIA&t=1452

  auto p_ball = getFutureBallPosition(ball.pos, ball.vel, time);
  if (robots.empty()) {
    return std::nullopt;
  }

  Point intercept_point = p_ball + ball.vel.normalized() * 0.3;

  // 各ロボットの移動時間を計算し、その中で最小のものを選ぶ
  auto best_robot = ranges::min(
    robots | ranges::views::transform([&](const auto & robot) {
      return std::make_pair(
        robot, getTravelTimeTrapezoidal(robot, intercept_point, max_acc, max_vel));
    }),
    ranges::less{}, [](const auto & pair) {
      return pair.second;  // 移動時間が小さい順にソート
    });

  return std::make_optional<SlackTimeResult>(
    {time - best_robot.second, intercept_point, best_robot.first});
}

auto WorldModelWrapper::getBallSequence(double t_horizon, double t_step)
  -> std::vector<std::pair<Point, double>>
{
  std::vector<double> t_ball_sequence = generateSequence(0.0, t_horizon, t_step);
  std::vector<std::pair<Point, double>> ball_sequence;

  std::optional<Point> intercepted_point = std::nullopt;
  for (auto t_ball : t_ball_sequence) {
    auto p_ball = getFutureBallPosition(ball.pos, ball.vel, t_ball, 1.0);
    if (not intercepted_point) {
      auto our_robots = ours.getAvailableRobots();
      auto their_robots = theirs.getAvailableRobots();
      auto nearest_friend = getNearestRobotWithDistanceFromPoint(p_ball, our_robots);
      auto nearest_enemy = getNearestRobotWithDistanceFromPoint(p_ball, their_robots);
      if (
        (nearest_friend.has_value() && nearest_friend->distance < 0.2) or
        (nearest_enemy.has_value() && nearest_enemy->distance < 0.2)) {
        intercepted_point = p_ball;
      }
    }

    if (intercepted_point) {
      ball_sequence.push_back({intercepted_point.value(), t_ball});
    } else {
      ball_sequence.push_back({p_ball, t_ball});
    }
  }
  return ball_sequence;
}

auto WorldModelWrapper::getSlackInterceptPointAndSlackTimeArray(
  const RobotList & robots, double t_horizon, double t_step, double slack_time_offset,
  const double max_acc, const double max_vel, double distance_horizon)
  -> std::vector<SlackTimeResult>
{
  auto ball_sequence = getBallSequence(t_horizon, t_step);
  auto their_robots = theirs.getAvailableRobots();
  // ボールの位置とスラックタイムをペアにして計算
  return ball_sequence
         // distance_horizon以内のボールのみを抽出
         | ranges::views::filter([&](const auto & ball_state) {
             return (ball_state.first - ball.pos).norm() < distance_horizon;
           })
         // フィールド外のボールを除外
         | ranges::views::filter(
             [&](const auto & ball_state) { return point_checker.isFieldInside(ball_state.first); })
         // 敵のブロックが入るまでのボールのみを抽出
         | ranges::views::take_while([&](const auto & ball_state) {
             auto nearest = getNearestRobotWithDistanceFromPoint(ball_state.first, their_robots);
             if (nearest.has_value()) {
               return nearest->distance > 0.2;
             } else {
               // 敵がいない場合は有効
               return true;
             }
           })
         // ボール位置 -> スラックタイムを計算
         | ranges::views::transform([&](const auto & ball_state) -> std::optional<SlackTimeResult> {
             auto [p_ball, t_ball] = ball_state;
             auto slack = getBallSlackTime(t_ball, robots, max_acc, max_vel);
             if (slack) {
               slack->slack_time += slack_time_offset;
             }
             return slack;
           })
         // 有効なスラックタイムのみを抽出
         |
         ranges::views::filter([&](const auto & opt_slack) {
           // 有効なスラックタイムかチェック
           return opt_slack.has_value() && point_checker.isFieldInside(opt_slack->intercept_point);
         }) |
         ranges::views::transform([](const auto & opt_pair) { return opt_pair.value(); }) |
         ranges::to<std::vector>();
}

auto WorldModelWrapper::getMinMaxSlackInterceptPointAndSlackTime(
  const RobotList & robots, double t_horizon, double t_step, double slack_time_offset,
  const double max_acc, const double max_vel, double distance_horizon)
  -> std::pair<std::optional<SlackTimeResult>, std::optional<SlackTimeResult>>
{
  auto slack_times = getSlackInterceptPointAndSlackTimeArray(
    robots, t_horizon, t_step, slack_time_offset, max_acc, max_vel, distance_horizon);
  if (ranges::empty(slack_times)) {
    return {std::nullopt, std::nullopt};
  }

  // min_slackはボールにできるだけ近い有効な位置
  std::optional<SlackTimeResult> min_slack = std::nullopt;
  for (const auto & slack : slack_times) {
    if (slack.slack_time > 0.0) {
      min_slack = slack;
      break;
    }
  }

  // max_slackは名前の通り一番Slackが大きい位置
  auto max_slack = ranges::max(
    slack_times, ranges::less{}, [](const auto & opt_pair) { return opt_pair.slack_time; });

  return {min_slack, max_slack};
}

auto WorldModelWrapper::BallOwnerCalculator::update() -> void
{
  double ball_distance_horizon = world_model->getMsg().game_analysis.ball_horizon;
  updateScore(true, ball_distance_horizon);
  updateScore(false, ball_distance_horizon);

  uint8_t our_frontier_old = std::exchange(our_frontier, [&]() { return our_frontier; }());
}

auto WorldModelWrapper::BallOwnerCalculator::updateScore(
  bool our_team, double ball_distance_horizon) -> void
{
  auto robots = our_team ? world_model->ours.getAvailableRobots(world_model->getOurGoalieId())
                         : world_model->theirs.getAvailableRobots();

  // ロボットのスコアを計算
  auto scores = robots | ranges::views::transform([&](const std::shared_ptr<RobotInfo> & robot) {
                  return calculateScore(robot, ball_distance_horizon);
                }) |
                ranges::to<std::vector>();

  // スコアの高い順にソート
  ranges::sort(
    scores, [](const RobotWithScore & a, const RobotWithScore & b) { return a.score > b.score; });

  if (our_team) {
    sorted_our_robots = std::move(scores);
  } else {
    sorted_their_robots = std::move(scores);
  }
}

auto WorldModelWrapper::BallOwnerCalculator::calculateScore(
  const std::shared_ptr<RobotInfo> & robot, double ball_distance_horizon) const
  -> WorldModelWrapper::BallOwnerCalculator::RobotWithScore
{
  RobotWithScore score;
  score.robot = robot;
  auto [min_slack, max_slack] = world_model->getMinMaxSlackInterceptPointAndSlackTime(
    {robot}, 3.0, 0.1, 0.5, 3.0, 5.0, ball_distance_horizon);
  if (min_slack.has_value() && min_slack.value().slack_time > 0.) {
    score.min_slack = min_slack->slack_time;
    score.min_slack_pos_distance = (min_slack->intercept_point - world_model->ball.pos).norm();
    // min_slackが正（間に合う）ならボールに近いほうがスコアが高い
    score.score = 100 - score.min_slack_pos_distance;
  } else {
    score.min_slack = 100.;
    score.min_slack_pos_distance = 100.;
    if (max_slack.has_value() && max_slack.value().slack_time > 0.) {
      // 間に合わない場合は、max_slackが大きいほうがスコアが高い
      score.score = max_slack.value().slack_time;
    } else {
      // どちらも間に合わない場合はスコアが低い
      score.score = -100. - robot->getDistance(world_model->ball.pos);
    }
  }
  if (max_slack.has_value()) {
    score.max_slack = max_slack->slack_time;
  } else {
    score.max_slack = -100.;
  }

  return score;
}
}  // namespace crane
