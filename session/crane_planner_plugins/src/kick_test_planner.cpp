// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/kick_test_planner.hpp>
#include <range/v3/all.hpp>

namespace crane
{
KickTestPlanner::KickTestPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
: PlannerBase("kick_test", world_model),
  kick_power("kick_test/kick_power", node, 0.0),
  chip_enable("kick_test/chip_enable", node, false),
  dribble_power("kick_test/dribble_power", node, 0.0),
  target_x("kick_test/target_x", node, 0.0),
  max_vel("kick_test/max_vel", node, 1.0)
{
}

auto KickTestPlanner::calculateRobotCommand(
  const std::vector<RobotIdentifier> & robots, PlannerContext &)
  -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>>
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
    auto command =
      std::make_shared<crane::RobotCommandWrapper>("kick_test", robot_id->id, world_model);
    auto robot = world_model->getRobot(*robot_id);

    if (chip_enable.getValue()) {
      command->kickWithChip(kick_power.getValue());
    } else {
      command->kickStraight(kick_power.getValue());
    }
    command->dribble(dribble_power.getValue());
    command->setMaxVelocity(max_vel.getValue());
    command->setTargetPosition(Point(target_x.getValue(), 0.0));

    robot_commands.emplace_back(command->getMsg());
  }

  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto KickTestPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
  -> std::vector<uint8_t>
{
  return this->getSelectedRobotsByScore(
    selectable_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // choose id smaller first
      return 15. - static_cast<double>(robot->id);
    },
    prev_roles, context);
}
}  // namespace crane
