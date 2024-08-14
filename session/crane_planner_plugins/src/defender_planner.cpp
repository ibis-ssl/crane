// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/defender_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
DefenderPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  if (robots.empty()) {
    return {PlannerBase::Status::RUNNING, {}};
  }

  auto ball = world_model->ball.pos;
  [[maybe_unused]] const double OFFSET_X = 0.1;
  [[maybe_unused]] const double OFFSET_Y = 0.1;

  //
  // calc ball line
  //
  Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
  {
    // シュート判定
    auto goal_posts = world_model->getOurGoalPosts();
    Segment goal_line(goal_posts.first, goal_posts.second);
    auto intersections = getIntersections(ball_line, goal_line);
    if (intersections.empty()) {
      // シュートがなければ通常の動き
      ball_line.first = world_model->getOurGoalCenter();
      ball_line.second = ball;
    }
  }

  std::vector<Point> defense_points = [&]() {
    // フィールド横幅の半分よりボールが遠ければ円弧守備に移行
    if (
      world_model->getDistanceFromBall(world_model->getOurGoalCenter()) <
      world_model->field_size.y() * 0.5) {
      return getDefenseLinePoints(robots.size(), ball_line);
    } else {
      return getDefenseArcPoints(robots.size(), ball_line);
    }
  }();

  if (not defense_points.empty()) {
    std::vector<Point> robot_points;
    for (auto robot_id : robots) {
      robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
    }

    auto solution = getOptimalAssignments(robot_points, defense_points);

    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
      int index = std::distance(robots.begin(), robot_id);
      Point target_point = defense_points[index];

      auto command = std::make_shared<crane::RobotCommandWrapperPosition>(
        "defender_planner", robot_id->robot_id, world_model);
      auto robot = world_model->getRobot(*robot_id);

      command->setTargetPosition(target_point);
      command->setTargetTheta(getAngle(world_model->ball.pos - target_point));
      command->disableCollisionAvoidance();
      command->disableBallAvoidance();

      robot_commands.emplace_back(command->getMsg());
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  } else {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
      int index = std::distance(robots.begin(), robot_id);
      [[maybe_unused]] Point target_point = [&]() {
        if (not defense_points.empty()) {
          return defense_points.at(index);
        } else {
          return Point(0, 0);
        }
      }();

      auto command = std::make_shared<crane::RobotCommandWrapperPosition>(
        "defender_planner/stop", robot_id->robot_id, world_model);

      auto robot = world_model->getRobot(*robot_id);

      // Stop at same position
      command->stopHere();

      robot_commands.emplace_back(command->getMsg());
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }
}

std::vector<Point> DefenderPlanner::getDefenseArcPoints(
  const int robot_num, const Segment & ball_line) const
{
  const double DEFENSE_INTERVAL = 0.5;
  const double RADIUS_OFFSET = 0.5;
  std::vector<Point> defense_points;
  // ペナルティエリアの一番遠い点を通る円の半径
  const double RADIUS =
    std::hypot(world_model->penalty_area_size.x(), world_model->penalty_area_size.y() * 0.5) +
    RADIUS_OFFSET;
  // r * theta = interval
  // theta = interval / e
  const double ANGLE_INTERVAL = DEFENSE_INTERVAL / RADIUS;

  auto defense_point = [&]() -> Point {
    Circle circle;
    circle.center = world_model->getOurGoalCenter();
    circle.radius = RADIUS;
    auto intersections = getIntersections(circle, ball_line);
    switch (static_cast<int>(intersections.size())) {
      case 0: {
        // ボールの進行方向がこちらを向いていないときは、中間地点に潜り込む
        return world_model->getOurGoalCenter() +
               (world_model->ball.pos - world_model->getOurGoalCenter()).normalized() * RADIUS;
      }
      case 1: {
        return intersections[0];
      }
      default: {
        // ボールに一番近い交点を返す
        double min_distance = std::numeric_limits<double>::max();
        Point best_intersection =
          world_model->getOurGoalCenter() +
          (world_model->ball.pos - world_model->getOurGoalCenter()).normalized() * RADIUS;
        for (auto & intersection : intersections) {
          double distance = (world_model->ball.pos, intersection).norm();
          if (distance < min_distance) {
            min_distance = distance;
            best_intersection = intersection;
          }
        }
        return best_intersection;
      }
    }
  }();

  double defense_angle = getAngle(defense_point - world_model->getOurGoalCenter());
  for (int i = 0; i < robot_num; i++) {
    double normalized_angle_offset = (robot_num - i - 1) / 2.;
    defense_points.emplace_back(
      world_model->getOurGoalCenter() +
      getNormVec(defense_angle + ANGLE_INTERVAL * normalized_angle_offset) * RADIUS);
  }
  return defense_points;
}

std::vector<Point> DefenderPlanner::getDefenseLinePoints(
  const int robot_num, const Segment & ball_line) const
{
  const double DEFENSE_INTERVAL = 0.2;
  std::vector<Point> defense_points;

  if (auto defense_parameter = getDefenseLinePointParameter(ball_line)) {
    double upper_parameter = *defense_parameter, lower_parameter = *defense_parameter;

    auto add_parameter = [&](double parameter) -> bool {
      const double OFFSET_X = 0.1, OFFSET_Y = 0.1;
      auto [threshold1, threshold2, threshold3] =
        getDefenseLinePointParameterThresholds(OFFSET_X, OFFSET_Y);
      if (parameter < 0. || parameter > threshold3) {
        return false;
      } else {
        if (upper_parameter < parameter) {
          upper_parameter = parameter;
        }
        if (lower_parameter > parameter) {
          lower_parameter = parameter;
        }
        defense_points.push_back(getDefenseLinePoint(parameter));
        return true;
      }
    };
    // 1台目
    upper_parameter = *defense_parameter;
    lower_parameter = *defense_parameter;
    add_parameter(*defense_parameter);

    // 2台目以降
    for (int i = 0; i < robot_num - 1; i++) {
      if (i % 2 == 0) {
        // upper側に追加
        if (not add_parameter(upper_parameter + DEFENSE_INTERVAL)) {
          // だめならlower側
          add_parameter(lower_parameter - DEFENSE_INTERVAL);
        }
      } else {
        // lower側に追加
        if (not add_parameter(lower_parameter - DEFENSE_INTERVAL)) {
          // だめならupper側
          add_parameter(upper_parameter + DEFENSE_INTERVAL);
        }
      }
    }
  }

  return defense_points;
}
}  // namespace crane
