// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/their_penalty_kick_session.hpp>

namespace crane
{
std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
TheirPenaltyKickSession::calculatePositionCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  // ボールより敵ゴール寄りに1m以上離し、フィールド幅に均等分散（ルール5.3.5.3）
  const double field_half_width = world_model->fieldSize().y() * 0.5;
  const double ball_x = world_model->ball().pos.x();
  const double their_goal_x = world_model->getTheirGoalCenter().x();
  const double sign = (their_goal_x > ball_x) ? 1.0 : -1.0;
  const double target_x = ball_x + sign * 1.1;
  const size_t n = other_robots.size();
  for (size_t i = 0; i < n; ++i) {
    auto & command = other_robots[i];
    Point target{};
    double target_y = (n > 1) ? -field_half_width + 2.0 * field_half_width * i / (n - 1) : 0.0;
    target << target_x, target_y;
    command->setTargetPosition(target);
    command->enableBallAvoidance();
    command->enableGoalAreaAvoidance();
    command->setMaxVelocity("TheirPenaltyKickSession for non goalie", 1.5);
    robot_commands.push_back(command->getMsg());
  }
  if (goalie) {
    if (
      world_model->getMsg().play_situation.command.value ==
      crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION) {
      auto & cmd = goalie->commander();
      cmd->setTargetPosition(world_model->getOurGoalCenter());
      cmd->lookAtBall();
      cmd->setMaxVelocity("TheirPenaltyKickSession for goalie", 1.5);
      cmd->disableAnyAreaAvoidance();
    } else {
      [[maybe_unused]] auto status = goalie->run();
    }
    robot_commands.emplace_back(goalie->getRobotCommand());
  }
  return {SessionBase::Status::RUNNING, robot_commands};
}

}  // namespace crane
