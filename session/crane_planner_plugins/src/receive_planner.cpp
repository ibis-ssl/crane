// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/receive_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
ReceivePlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  if (not receiver_skill) {
    return {PlannerBase::Status::RUNNING, {}};
  } else {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    auto status = receiver_skill->run(visualizer);
    return {static_cast<PlannerBase::Status>(status), {receiver_skill->getRobotCommand()}};
  }
}
}  // namespace crane
