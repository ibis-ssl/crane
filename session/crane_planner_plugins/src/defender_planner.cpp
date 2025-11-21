// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/defender_planner.hpp>
#include <range/v3/all.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
DefenderPlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> & robots, PlannerContext &)
{
  if (robots.empty()) {
    return {PlannerBase::Status::RUNNING, {}};
  }

  auto ball = world_model->ball().pos;

  //
  // calc ball line
  //
  Segment ball_line(ball, ball + world_model->ball().vel.normalized() * 20.f);
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
      world_model->fieldSize().y() * 0.5) {
      return getDefenseLinePoints(robots.size(), ball_line, world_model);
    } else {
      return getDefenseArcPoints(robots.size(), ball_line, world_model);
    }
  }();

  if (not defense_points.empty()) {
    auto robot_commands = assignRobotsToPoints(
      robots, defense_points, "defender_planner", world_model->ball().pos,
      [&](std::shared_ptr<RobotCommandWrapper> & command) {
        command->disableBasicAvoidances();
        if (
          world_model->getMsg().play_situation.command.value ==
          crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT) {
          command->disableAnyAreaAvoidance();
          command->enablePlacementAvoidance();
        } else {
          command->disableAnyAreaAvoidance();
          command->enableGoalAreaAvoidance();
        }
      });
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

      auto command = std::make_shared<crane::RobotCommandWrapper>(
        "defender_planner/stop", robot_id->id, world_model);

      auto robot = world_model->getRobot(*robot_id);

      // Stop at same position
      command->stopHere();

      robot_commands.emplace_back(command->getMsg());
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }
}
}  // namespace crane
