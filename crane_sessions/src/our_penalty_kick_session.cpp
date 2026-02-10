// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/our_penalty_kick_session.hpp>

namespace crane
{
std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
OurPenaltyKickSession::calculatePositionCommand(const std::vector<RobotIdentifier> &)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  for (auto & command : other_robots) {
    // 関係ないロボットはボールより1m以上下がる(ルール5.3.5.3)
    Point target{};
    target << (world_model->getOurGoalCenter().x() + world_model->ball().pos.x()) / 2,
      command->getRobot()->pose.pos.y();
    command->setTargetPosition(target);
    command->setMaxVelocity("OurPenaltyKickSession", 0.5);
    command->enableBallAvoidance();
    robot_commands.push_back(command->getMsg());
  }
  if (kicker) {
    auto status = kicker->run();
    robot_commands.emplace_back(kicker->getRobotCommand());
    if (status == skills::Status::SUCCESS) {
      return {Status::SUCCESS, robot_commands};
    }
  }
  return {Status::RUNNING, robot_commands};
}
}  // namespace crane
