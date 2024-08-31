// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/their_penalty_kick_planner.hpp>

namespace crane
{

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
TheirPenaltyKickPlanner::calculateRobotCommand(
  [[maybe_unused]] const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  for (auto & command : other_robots) {
    // 関係ないロボットはボールより1m以上下がる(ルール5.3.5.3)
    Point target{};
    target << (world_model->getTheirGoalCenter().x() + world_model->ball.pos.x()) / 2,
      command->getRobot()->pose.pos.y();
    command->setTargetPosition(target);
    command->disableGoalAreaAvoidance();
    command->disableRuleAreaAvoidance();
    command->setMaxVelocity(1.5);
    robot_commands.push_back(command->getMsg());
  }
  if (goalie) {
    if (
      world_model->play_situation.getSituationCommandID() ==
      crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION) {
      auto & cmd = goalie->commander();
      cmd.setTargetPosition(world_model->getOurGoalCenter());
      cmd.lookAtBall();
      cmd.setMaxVelocity(1.5);
      cmd.disableRuleAreaAvoidance();
      cmd.disableGoalAreaAvoidance();
    } else {
      [[maybe_unused]] auto status = goalie->run(visualizer);
    }
    robot_commands.emplace_back(goalie->getRobotCommand());
  }
  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto TheirPenaltyKickPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto goalie_base = std::make_shared<RobotCommandWrapperBase>(
    "their_penalty_kick_planner/goalie", world_model->getOurGoalieId(), world_model);
  goalie = std::make_shared<skills::Goalie>(goalie_base);
  auto robots_sorted = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [&](const std::shared_ptr<RobotInfo> & robot) {
      // ボールに近いほうが先頭
      return 100. / robot->getDistance(world_model->ball.pos);
    },
    prev_roles);
  for (auto it = robots_sorted.begin(); it != robots_sorted.end(); it++) {
    if (*it != world_model->getOurGoalieId()) {
      other_robots.emplace_back(std::make_shared<RobotCommandWrapperPosition>(
        "their_penalty_kick_planner", *it, world_model));
    }
  }
  return robots_sorted;
}
}  // namespace crane
