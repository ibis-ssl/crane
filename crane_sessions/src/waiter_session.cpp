// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/waiter_session.hpp>

namespace crane
{
std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
WaiterSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  for (auto robot_id : robots) {
    auto command =
      std::make_shared<crane::PositionCommandWrapper>("waiter_planner", robot_id.id, world_model);
    command->stopHere();
    command->disableAnyAreaAvoidance();
    command->disableCollisionAvoidance();
    robot_commands.emplace_back(command->getMsg());
  }
  return {SessionBase::Status::RUNNING, robot_commands};
}

}  // namespace crane
