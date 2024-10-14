// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/test_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
TestPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  static int count = 0;
  count++;
  for (auto robot_id : robots) {
    auto command = std::make_shared<crane::RobotCommandWrapperPosition>(
      "test_planner", robot_id.robot_id, world_model);
    if (count % (60 * 10) > 60 * 5) {
      command->setTargetPosition(Point(2.0, 2.0));
    } else {
      command->setTargetPosition(Point(2.0, -2.0));
    }
    robot_commands.emplace_back(command->getMsg());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto TestPlanner::getSelectedRobots(
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
  return selected;
}
}  // namespace crane
