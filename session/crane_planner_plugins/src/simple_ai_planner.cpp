// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/simple_ai_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
SimpleAIPlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> & robots, PlannerContext & context)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  if (running_skill) {
    skill_status = running_skill->run();
    robot_commands.push_back(running_skill->getRobotCommand());
  }

  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto SimpleAIPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  -> std::vector<uint8_t>
{
  auto selected = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // choose id smaller first
      return 15. - static_cast<double>(-robot->id);
    },
    prev_roles, context);
  return selected;
}
}  // namespace crane
