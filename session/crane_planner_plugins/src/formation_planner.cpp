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
  formation_points.emplace_back(1.0, 0.5);
  formation_points.emplace_back(1.0, -0.5);
  formation_points.emplace_back(2.0, 0.0);
  formation_points.emplace_back(2.0, 1.5);
  formation_points.emplace_back(2.0, -1.5);
  formation_points.emplace_back(3.0, 3.0);
  formation_points.emplace_back(3.0, 1.0);
  formation_points.emplace_back(3.0, -1.0);
  formation_points.emplace_back(3.0, -3.0);
  formation_points.emplace_back(4.0, 1.5);
  formation_points.emplace_back(4.0, 0.0);
  formation_points.emplace_back(4.0, -1.5);

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
    Point target_point = formation_points[solution[index]];

    auto command = std::make_shared<crane::RobotCommandWrapperPosition>(
      "formation_planner", robot_id->robot_id, world_model);

    command->setTargetPosition(target_point);
    command->setTargetTheta(target_theta);
    command->setMaxVelocity(1.0);

    robot_commands.emplace_back(command->getMsg());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto FormationPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  return this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // choose id smaller first
      return 15. - static_cast<double>(-robot->id);
    },
    prev_roles);
}
}  // namespace crane
