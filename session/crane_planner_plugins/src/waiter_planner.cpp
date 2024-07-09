// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/waiter_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
WaiterPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (auto robot_id : robots) {
    crane::RobotCommandWrapperPosition target(robot_id.robot_id, world_model);
    target.stopHere();
    if (target.robot->vel.linear.norm() < 0.5) {
      target.stopEmergency();
    }
    robot_commands.emplace_back(target.getMsg());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto WaiterPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // choose id smaller first
      return 15. - static_cast<double>(-robot->id);
    },
    prev_roles);
  for (auto robot : selected) {
    stop_poses.emplace(robot, world_model->getOurRobot(robot)->pose);
  }
  return selected;
}
}  // namespace crane
