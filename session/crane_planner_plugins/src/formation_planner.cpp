// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/formation_planner.hpp>

namespace crane
{
std::vector<Point> FormationPlanner::getFormationPoints(int robot_num)
{
  std::vector<Point> formation_points;
  formation_points.emplace_back(0.6, 0.0);
  formation_points.emplace_back(0.9, 0.5);
  formation_points.emplace_back(0.9, -0.5);
  formation_points.emplace_back(1.6, 0.0);
  formation_points.emplace_back(1.6, 1.0);
  formation_points.emplace_back(1.6, -1.0);

  if (world_model->getOurGoalCenter().x() < 0.0) {
    for (auto & point : formation_points) {
      point.x() *= -1.0;
    }
  }

  formation_points.resize(robot_num);
  return formation_points;
}
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
FormationPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<Point> robot_points;
  for (auto robot_id : robots) {
    robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
  }
  auto formation_points = getFormationPoints(robots.size());

  auto solution = getOptimalAssignments(robot_points, formation_points);

  double target_theta = (world_model->getOurGoalCenter().x() > 0.0) ? M_PI : 0.0;
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
    int index = std::distance(robots.begin(), robot_id);
    Point target_point = formation_points[index];

    crane::RobotCommandWrapper target(robot_id->robot_id, world_model);
    target.setTargetPosition(target_point);
    target.setTargetTheta(target_theta);

    robot_commands.emplace_back(target.getMsg());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}
}  // namespace crane
