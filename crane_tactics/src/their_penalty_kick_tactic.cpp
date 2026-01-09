// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_tactics/their_penalty_kick_tactic.hpp>

namespace crane
{
std::pair<TacticBase::Status, std::vector<crane_msgs::msg::PositionCommand>>
TheirPenaltyKickTactic::calculatePositionCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::PositionCommand> robot_commands;

  for (auto & command : other_robots) {
    // 関係ないロボットはボールより1m以上下がる(ルール5.3.5.3)
    Point target{};
    target << (world_model->getTheirGoalCenter().x() + world_model->ball().pos.x()) / 2,
      command->getRobot()->pose.pos.y();
    command->setTargetPosition(target);
    command->disableAnyAreaAvoidance();
    command->enableBallAvoidance();
    command->setMaxVelocity("TheirPenaltyKickTactic for non goalie", 1.5);
    robot_commands.push_back(command->getMsg());
  }
  if (goalie) {
    if (
      world_model->getMsg().play_situation.command.value ==
      crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION) {
      auto & cmd = goalie->commander();
      cmd->setTargetPosition(world_model->getOurGoalCenter());
      cmd->lookAtBall();
      cmd->setMaxVelocity("TheirPenaltyKickTactic for goalie", 1.5);
      cmd->disableAnyAreaAvoidance();
    } else {
      [[maybe_unused]] auto status = goalie->run();
    }
    robot_commands.emplace_back(goalie->getRobotCommand());
  }
  return {TacticBase::Status::RUNNING, robot_commands};
}

}  // namespace crane
