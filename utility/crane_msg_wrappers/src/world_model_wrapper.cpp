// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/world_model_wrapper.hpp"

namespace crane
{
auto BallContact::update(bool is_contacted) -> void
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
    ours_.robots.emplace_back(std::make_shared<RobotInfo>());
    theirs_.robots.emplace_back(std::make_shared<RobotInfo>());
  }

  if (setup_subscriber) {
    subscriber = node.create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10,
      [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void { this->update(*msg); });
  }
}

auto WorldModelWrapper::update(const crane_msgs::msg::WorldModel & world_model) -> void
{
  has_updated = true;
  latest_msg = world_model;
  for (auto & our_robot : ours_.robots) {
    our_robot->available = false;
  }

  for (auto & their_robot : theirs_.robots) {
    their_robot->available = false;
  }

  ours_.max_allowed_bots = world_model.our_max_allowed_bots;
  theirs_.max_allowed_bots = world_model.their_max_allowed_bots;

  // BallInfoメッセージからBall構造体への変換
  ball_.fromMsg(world_model.ball_info);
  ball_.ball_speed_hysteresis.update(ball_.vel.norm());

  for (auto & robot : world_model.robot_info_ours) {
    auto & info = ours_.robots.at(robot.id);
    info->available = robot.detected;
    if (info->available) {
      info->id = robot.id;
      info->vision_detection_stamp = robot.vision.stamp;
      info->pose.pos << robot.pose.x, robot.pose.y;
      info->pose.theta = robot.pose.theta;
      info->vel.linear << robot.velocity.x, robot.velocity.y;
      info->ball_contact.update((info->kicker_center() - ball_.pos).norm() < 0.1);
      // ボールセンサは味方だけ
      info->ball_sensor = robot.ball_sensor;
      info->ball_sensor_stamp = robot.last_ball_sensor_stamp;
    } else {
      info->ball_contact.update(false);
    }
  }

  for (auto robot : world_model.robot_info_theirs) {
    auto & info = theirs_.robots.at(robot.id);
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

  ours_.goalie_id = world_model.our_goalie_id;
  theirs_.goalie_id = world_model.their_goalie_id;

  field_size_ << world_model.field_info.x, world_model.field_info.y;
  penalty_area_size_ << world_model.penalty_area_size.x, world_model.penalty_area_size.y;

  goal_size_ << world_model.goal_size.x, world_model.goal_size.y;
  goal_ << getOurSideSign() * fieldSize().x() * 0.5, 0.;

  if (onPositiveHalf()) {
    ours_.penalty_area.max_corner() << goal_.x(), goal_.y() + world_model.penalty_area_size.y / 2.;
    ours_.penalty_area.min_corner() << goal_.x() - world_model.penalty_area_size.x,
      goal_.y() - world_model.penalty_area_size.y / 2.;
  } else {
    ours_.penalty_area.max_corner() << goal_.x() + world_model.penalty_area_size.x,
      goal_.y() + world_model.penalty_area_size.y / 2.;
    ours_.penalty_area.min_corner() << goal_.x(), goal_.y() - world_model.penalty_area_size.y / 2.;
  }
  theirs_.penalty_area.max_corner()
    << std::max(-ours_.penalty_area.max_corner().x(), -ours_.penalty_area.min_corner().x()),
    ours_.penalty_area.max_corner().y();
  theirs_.penalty_area.min_corner()
    << std::min(-ours_.penalty_area.max_corner().x(), -ours_.penalty_area.min_corner().x()),
    ours_.penalty_area.min_corner().y();

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
  for (float x = 0.f; x <= fieldSize().x() / 2.f; x += grid_size) {
    for (float y = 0.f; y <= fieldSize().y() / 2.f; y += grid_size) {
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
  field_box.min_corner() << -world_model->fieldSize().x() / 2.f - offset,
    -world_model->fieldSize().y() / 2.f - offset;
  field_box.max_corner() << world_model->fieldSize().x() / 2.f + offset,
    world_model->fieldSize().y() / 2.f + offset;
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
  return isInBox(world_model->theirs_.penalty_area, p, offset);
}

auto WorldModelWrapper::PointChecker::isFriendPenaltyArea(const Point & p, double offset) const
  -> bool
{
  return isInBox(world_model->ours_.penalty_area, p, offset);
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
    area.segment.first = ball_.pos;
    area.segment.second = target.value();
    area.radius = 0.5 + offset;
    return area;
  } else {
    return std::nullopt;
  }
}

auto WorldModelWrapper::getLargestGoalAngleRangeFromPoint(
  const Point from, const std::pair<Point, Point> & goal_posts,
  const RobotList & obstacle_robots) const -> GoalAngleRange
{
  Interval goal_range;

  if (goal_posts.first.x() < 0.) {
    goal_range.append(
      normalizeAngle(getAngle(goal_posts.first - from) + M_PI),
      normalizeAngle(getAngle(goal_posts.second - from) + M_PI));
  } else {
    goal_range.append(getAngle(goal_posts.first - from), getAngle(goal_posts.second - from));
  }

  for (auto & obstacle : obstacle_robots) {
    double distance = obstacle->getDistance(from);
    constexpr double MACHINE_RADIUS = 0.1;

    double center_angle = [&]() {
      if (goal_posts.first.x() < 0.) {
        return normalizeAngle(getAngle(obstacle->pose.pos - from) + M_PI);
      } else {
        return getAngle(obstacle->pose.pos - from);
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

  if (std::abs(getAngleDiff(largest_interval.second, largest_interval.first)) < 0.0001) {
    target_angle =
      getIntermediateAngle(getAngle(goal_posts.first - from), getAngle(goal_posts.second - from));
  }

  return {target_angle, getAngleDiff(largest_interval.second, largest_interval.first)};
}

auto WorldModelWrapper::getBallSlackTime(
  double time, const RobotList & robots, const double max_acc, const double max_vel)
  -> std::optional<SlackTimeResult>
{
  // https://www.youtube.com/live/bizGFvaVUIk?si=mFZqirdbKDZDttIA&t=1452

  auto p_ball = ball_.getPredictedPosition(time);
  if (robots.empty()) {
    return std::nullopt;
  }

  // NaN値チェック
  if (!std::isfinite(p_ball.x()) || !std::isfinite(p_ball.y())) {
    std::cout << "WARN: [WorldModelWrapper] getBallSlackTime: p_ballがNaN値のため処理をスキップ" << std::endl;
    return std::nullopt;
  }

  Point intercept_point;
  if (ball_.vel.norm() < 1e-6) {
    // ボール速度がほぼゼロの場合、正規化を避けてボール位置を使用
    intercept_point = p_ball;
  } else {
    Vector2 normalized_vel = ball_.vel.normalized();
    // 正規化後のNaN値チェック
    if (!std::isfinite(normalized_vel.x()) || !std::isfinite(normalized_vel.y())) {
      std::cout << "WARN: [WorldModelWrapper] getBallSlackTime: normalized_velがNaN値のためp_ballを使用" << std::endl;
      intercept_point = p_ball;
    } else {
      intercept_point = p_ball + normalized_vel * 0.3;
    }
  }

  // intercept_pointのNaN値チェック
  if (!std::isfinite(intercept_point.x()) || !std::isfinite(intercept_point.y())) {
    std::cout << "WARN: [WorldModelWrapper] getBallSlackTime: intercept_pointがNaN値のため処理をスキップ" << std::endl;
    return std::nullopt;
  }

  // 各ロボットの移動時間を計算し、その中で最小のものを選ぶ
  auto best_robot = ranges::min(
    robots | ranges::views::transform([&](const auto & robot) {
      return std::make_pair(
        robot, getTravelTimeTrapezoidal(robot, intercept_point, max_acc, max_vel));
    }),
    ranges::less{}, [](const auto & pair) {
      return pair.second;  // 移動時間が小さい順にソート
    });

  double slack_time = time - best_robot.second;
  // slack_timeのNaN値チェック
  if (!std::isfinite(slack_time)) {
    std::cout << "WARN: [WorldModelWrapper] getBallSlackTime: slack_timeがNaN値のため処理をスキップ" << std::endl;
    return std::nullopt;
  }

  return std::make_optional<SlackTimeResult>(
    {slack_time, intercept_point, best_robot.first});
}

auto WorldModelWrapper::getBallSequence(double t_horizon, double t_step)
  -> std::vector<std::pair<Point, double>>
{
  return ball_.getBallSequence(t_horizon, t_step);
}

auto WorldModelWrapper::getSlackInterceptPointAndSlackTimeArray(
  const RobotList & robots, double t_horizon, double t_step, double slack_time_offset,
  const double max_acc, const double max_vel, double distance_horizon, double velocity_epsilon)
  -> std::vector<SlackTimeResult>
{
  std::vector<std::pair<Point, double>> ball_sequence;
  if (ball_.vel.norm() > velocity_epsilon) {
    ball_sequence = getBallSequence(t_horizon, t_step);
  } else {
    ball_sequence.emplace_back(ball_.pos, 0.0);
  }
  auto their_robots = theirs_.getAvailableRobots();
  // ボールの位置とスラックタイムをペアにして計算
  return ball_sequence
         // distance_horizon以内のボールのみを抽出
         | ranges::views::filter([&](const auto & ball_state) {
             return (ball_state.first - ball_.pos).norm() < distance_horizon;
           })
         // フィールド外/ペナルティエリア内のボールを除外
         | ranges::views::filter([&](const auto & ball_state) {
             return point_checker.isFieldInside(ball_state.first) &&
                    not point_checker.isPenaltyArea(ball_state.first);
           })
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
  const double max_acc, const double max_vel, double distance_horizon, double velocity_epsilon)
  -> std::pair<std::optional<SlackTimeResult>, std::optional<SlackTimeResult>>
{
  auto slack_times = getSlackInterceptPointAndSlackTimeArray(
    robots, t_horizon, t_step, slack_time_offset, max_acc, max_vel, distance_horizon,
    velocity_epsilon);
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
  auto robots = our_team ? world_model->ours_.getAvailableRobots(world_model->getOurGoalieId())
                         : world_model->theirs_.getAvailableRobots();

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
    {robot}, 4.0, 0.1, 0.5, 2.5, 5.0, ball_distance_horizon);
  if (min_slack.has_value() && min_slack.value().slack_time > 0.) {
    score.min_slack = min_slack->slack_time;
    score.min_slack_pos_distance = (min_slack->intercept_point - world_model->ball().pos).norm();
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
      score.score = -100. - robot->getDistance(world_model->ball().pos);
    }
  }

  return score;
}
auto WorldModelWrapper::getPenaltyAreaCorners(double offset_x, double offset_y) const
  -> std::tuple<Point, Point, Point, Point>
{
  // ディフェンスエリアを囲みし4つの点
  Point p1;
  p1 << goal_.x() + std::copysign(0.5, goal_.x()), penalty_area_size_.y() * 0.5 + offset_y;
  Point p2 = p1;
  if (goal_.x() > 0) {
    p2.x() -= (penalty_area_size_.x() + offset_x + 0.5);
  } else {
    p2.x() += (penalty_area_size_.x() + offset_x + 0.5);
  }

  Point p3(p2.x(), -p2.y());
  Point p4(p1.x(), p3.y());
  return {p1, p2, p3, p4};
}

auto WorldModelWrapper::getOurAreaCorners() const -> std::tuple<Point, Point, Point, Point>
{
  const double field_size_y = fieldSize().y();
  Point p1;
  p1 << goal_.x(), field_size_y * 0.5;
  Point p2 = p1;
  p2.x() = 0.0;
  Point p3(p2.x(), -p2.y());
  Point p4(p1.x(), p3.y());
  return {p1, p2, p3, p4};
}

auto WorldModelWrapper::getIntersectionOurPenaltyArea(
  const Segment & target_segment, double offset_x, double offset_y) const -> std::optional<Point>
{
  auto [p1, p2, p3, p4] = getPenaltyAreaCorners(offset_x, offset_y);
  if (auto intersections = getIntersections(Segment{p1, p2}, target_segment);
      not intersections.empty()) {
    return intersections[0];
  } else if (intersections = getIntersections(Segment{p2, p3}, target_segment);
             not intersections.empty()) {
    return intersections[0];
  } else if (intersections = getIntersections(Segment{p3, p4}, target_segment);
             not intersections.empty()) {
    return intersections[0];
  } else if (intersections = getIntersections(Segment{p4, p1}, target_segment);
             not intersections.empty()) {
    return intersections[0];
  } else {
    return std::nullopt;
  }
}
auto WorldModelWrapper::getForwardDefenseRatio(const Segment & ball_line) const
  -> std::optional<double>
{
  const Vector2 segment_vec = (ball_line.second - ball_line.first).normalized();
  const auto ball_line_long_behind = Segment(ball_line.first - segment_vec * 20, ball_line.second);
  const auto ball_line_long_forward = Segment(ball_line.first, ball_line.second + segment_vec * 20);

  auto get_intersection_to_area =
    [](
      const Segment & target_segment,
      std::tuple<const Point &, const Point &, const Point &, const Point &> areas)
    -> std::optional<Point> {
    if (auto intersections =
          getIntersections(Segment{std::get<0>(areas), std::get<1>(areas)}, target_segment);
        not intersections.empty()) {
      return intersections[0];
    } else if (intersections =
                 getIntersections(Segment{std::get<1>(areas), std::get<2>(areas)}, target_segment);
               not intersections.empty()) {
      return intersections[0];
    } else if (intersections =
                 getIntersections(Segment{std::get<2>(areas), std::get<3>(areas)}, target_segment);
               not intersections.empty()) {
      return intersections[0];
    } else {
      return std::nullopt;
    }
  };

  const auto intersect_to_penalty_area =
    getIntersectionOurPenaltyArea(ball_line_long_forward, 0.0, 0.0);
  if (not intersect_to_penalty_area) {
    return std::nullopt;
  }

  auto [our_area_p1, our_area_p2, our_area_p3, our_area_p4] = getOurAreaCorners();
  const auto intersect_to_field_area =
    getIntersectionOurPenaltyArea(ball_line_long_behind, 0.0, 0.0);
  if (not intersect_to_field_area) {
    return std::nullopt;
  }

  double distance_ball_to_penalty_area = bg::distance(ball_.pos, intersect_to_penalty_area.value());
  double distance_ball_to_field_area = bg::distance(ball_.pos, intersect_to_field_area.value());
  double distance_sum = distance_ball_to_penalty_area + distance_ball_to_field_area;

  // ボールからペナルティエリアまでの距離が小さいほど大きな値が返る。
  return distance_ball_to_field_area / distance_sum;
}
}  // namespace crane
