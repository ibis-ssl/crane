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
  const double OFFSET_X = 0.1;
  const double OFFSET_Y = 0.1;

  //
  // calc ball line
  //
  Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
  {
    // シュート判定
    auto goal_posts = world_model->getOurGoalPosts();
    Segment goal_line(goal_posts.first, goal_posts.second);
    std::vector<Point> intersections;
    bg::intersection(ball_line, goal_line, intersections);
    if (intersections.empty()) {
      // シュートがなければ通常の動き
      ball_line.first = world_model->goal;
      ball_line.second = ball;
    }
  }

  auto first_defenders = robots;
  if (second_threat_defender && not world_model->theirs.getAvailableRobots().empty()) {
    first_defenders.erase(
      std::remove_if(
        first_defenders.begin(), first_defenders.end(),
        [&](const auto & robot_id) { return robot_id.robot_id == second_threat_defender.value(); }),
      first_defenders.end());
  }

  std::vector<Point> defense_points = getDefensePoints(first_defenders.size(), ball_line);

  if (not defense_points.empty()) {
    std::vector<Point> robot_points;
    for (auto robot_id : first_defenders) {
      robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
    }

    auto solution = getOptimalAssignments(robot_points, defense_points);

    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto robot_id = first_defenders.begin(); robot_id != first_defenders.end(); ++robot_id) {
      int index = std::distance(first_defenders.begin(), robot_id);
      Point target_point = defense_points[index];

      crane::RobotCommandWrapper target(robot_id->robot_id, world_model);
      auto robot = world_model->getRobot(*robot_id);

      target.setTargetPosition(target_point);
      target.setTargetTheta(getAngle(world_model->ball.pos - target_point));
      target.disableCollisionAvoidance();
      target.disableBallAvoidance();

      robot_commands.emplace_back(target.getMsg());
    }

    if (second_threat_defender && not world_model->theirs.getAvailableRobots().empty()) {
      auto ball_handler = world_model->getNearestRobotsWithDistanceFromPoint(
        world_model->ball.pos, world_model->theirs.getAvailableRobots());
      auto enemy_robots = world_model->theirs.getAvailableRobots(ball_handler.first->id);
      if (enemy_robots.empty()) {
        crane::RobotCommandWrapper target(second_threat_defender.value(), world_model);
        target.stopHere();
        robot_commands.emplace_back(target.getMsg());
      } else {
        double max_goal_width = 0.;
        uint8_t second_threat_bot_id = 0;
        for (const auto enemy : enemy_robots) {
          double goal_width =
            world_model->getLargestOurGoalAngleRangeFromPoint(enemy->pose.pos).second;
          if (goal_width > max_goal_width) {
            max_goal_width = goal_width;
            second_threat_bot_id = enemy->id;
          }
        }
        auto second_threat_bot = world_model->getRobot({false, second_threat_bot_id});
        Point mark_point = second_threat_bot->pose.pos +
                           (world_model->goal - second_threat_bot->pose.pos).normalized() * 0.3;
        crane::RobotCommandWrapper target(second_threat_defender.value(), world_model);
        target.setTargetPosition(mark_point);
        target.setTargetTheta(getAngle(second_threat_bot->pose.pos - mark_point));
        robot_commands.emplace_back(target.getMsg());
      }
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  } else {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto robot_id = first_defenders.begin(); robot_id != first_defenders.end(); ++robot_id) {
      int index = std::distance(first_defenders.begin(), robot_id);
      Point target_point = defense_points[index];

      crane::RobotCommandWrapper target(robot_id->robot_id, world_model);

      auto robot = world_model->getRobot(*robot_id);

      // Stop at same position
      target.stopHere();

      robot_commands.emplace_back(target.getMsg());
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }
}
}  // namespace crane
