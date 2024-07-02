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
    auto intersections = getIntersections(ball_line, goal_line);
    if (intersections.empty()) {
      // シュートがなければ通常の動き
      ball_line.first = world_model->goal;
      ball_line.second = ball;
    }
  }

  std::vector<Point> defense_points = getDefensePoints(robots.size(), ball_line);

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

      crane::RobotCommandWrapper target(robot_id->robot_id, world_model);
      auto robot = world_model->getRobot(*robot_id);

      target.setTargetPosition(target_point);
      target.setTargetTheta(getAngle(world_model->ball.pos - target_point));
      target.disableCollisionAvoidance();
      target.disableBallAvoidance();

      robot_commands.emplace_back(target.getMsg());
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  } else {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
      int index = std::distance(robots.begin(), robot_id);
      Point target_point = defense_points.at(index);

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
