// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/our_kickoff_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
OurKickOffPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;

  kickoff_attack->run(visualizer);
  robot_commands.emplace_back(kickoff_attack->getRobotCommand());
  if (kickoff_support) {
    kickoff_support->run(visualizer);
    robot_commands.emplace_back(kickoff_support->getRobotCommand());
  }

  // いい感じにSUCCESSも返す
  return {PlannerBase::Status::RUNNING, robot_commands};
}
auto OurKickOffPlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  // 一番ボールに近いロボットをkickoff attack
  auto best_attacker = std::max_element(
    selectable_robots.begin(), selectable_robots.end(), [this](const auto & a, const auto & b) {
      return world_model->getOurRobot(a)->getDistance(world_model->ball.pos) >
             world_model->getOurRobot(b)->getDistance(world_model->ball.pos);
    });
  Point supporter_pos{0.0, 3.0};
  auto best_supporter = std::max_element(
    selectable_robots.begin(), selectable_robots.end(),
    [this, supporter_pos, best_attacker](const auto & a, const auto & b) {
      if (a == *best_attacker) {
        // bの方大きい => best_attackerであるaが除外される
        return true;
      } else if (b == *best_attacker) {
        // bの方大きくない => best_attackerであるbが除外される
        return false;
      } else {
        return world_model->getOurRobot(a)->getDistance(supporter_pos) >
               world_model->getOurRobot(b)->getDistance(supporter_pos);
      }
    });

  kickoff_attack = std::make_shared<skills::KickoffAttack>(*best_attacker, world_model);
  if (*best_attacker != *best_supporter) {
    kickoff_support = std::make_shared<skills::KickoffSupport>(*best_supporter, world_model);
    kickoff_support->setParameter("target_x", supporter_pos.x());
    kickoff_support->setParameter("target_y", supporter_pos.y());
  }

  return {*best_attacker, *best_supporter};
}
}  // namespace crane
