// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/our_penalty_kick_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
OurPenaltyKickPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  for (auto & command : other_robots) {
    // 関係ないロボットはボールより1m以上下がる(ルール5.3.5.3)
    Point target{};
    target << (world_model->getOurGoalCenter().x() + world_model->ball.pos.x()) / 2,
      command->getRobot()->pose.pos.y();
    command->setTargetPosition(target);
    command->setMaxVelocity(0.5);
    robot_commands.push_back(command->getMsg());
  }
  if (kicker) {
    auto status = kicker->run(visualizer);
    robot_commands.emplace_back(kicker->getRobotCommand());
    if (status == skills::Status::SUCCESS) {
      return {Status::SUCCESS, robot_commands};
    }
  }
  return {Status::RUNNING, robot_commands};
}
auto OurPenaltyKickPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  auto robots_sorted = this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [&](const std::shared_ptr<RobotInfo> & robot) {
      // ボールに近いほうが先頭
      return 100. / robot->getDistance(world_model->ball.pos);
    },
    prev_roles);
  // ゴールキーパーはキッカーに含めない(ロボットがキーパーのみの場合は除く)
  if (robots_sorted.size() > 1 && robots_sorted.front() == world_model->getOurGoalieId()) {
    robots_sorted.erase(robots_sorted.begin());
  }
  if (robots_sorted.size() > 0) {
    // 一番ボールに近いロボットがキッカー
    auto kicker_base = std::make_shared<RobotCommandWrapperBase>(
      "our_penalty_kick_planner/kicker", robots_sorted.front(), world_model);
    kicker = std::make_shared<skills::PenaltyKick>(kicker_base);
  }
  if (robots_sorted.size() > 1) {
    for (auto it = robots_sorted.begin() + 1; it != robots_sorted.end(); it++) {
      other_robots.emplace_back(std::make_shared<RobotCommandWrapperPosition>(
        "our_penalty_kick_planner/other", *it, world_model));
    }
  }
  return robots_sorted;
}
}  // namespace crane
