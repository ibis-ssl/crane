// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__OUR_KICKOFF_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__OUR_KICKOFF_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/kickoff_attack.hpp>
#include <crane_robot_skills/kickoff_support.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class OurKickOffPlanner : public PlannerBase
{
private:
  std::shared_ptr<skills::KickoffAttack> kickoff_attack;

  std::shared_ptr<RobotCommandWrapper> attacker_command;

  std::shared_ptr<skills::KickoffSupport> kickoff_support;

  std::shared_ptr<RobotCommandWrapper> supporter_command;

public:
  COMPOSITION_PUBLIC explicit OurKickOffPlanner(
    WorldModelWrapper::SharedPtr & world_model, ConsaiVisualizerWrapper::SharedPtr visualizer)
  : PlannerBase("our_kickoff_planner", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;

    kickoff_attack->run(visualizer);
    kickoff_support->run(visualizer);

    robot_commands.emplace_back(kickoff_attack->getRobotCommand());
    robot_commands.emplace_back(kickoff_support->getRobotCommand());

    // いい感じにSUCCESSも返す
    return {PlannerBase::Status::RUNNING, robot_commands};
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    // 一番ボールに近いロボットをkickoff attack
    auto best_attacker = std::max_element(
      selectable_robots.begin(), selectable_robots.end(), [this](const auto & a, const auto & b) {
        return world_model->getOurRobot(a)->getDistance(world_model->ball.pos) <
               world_model->getOurRobot(b)->getDistance(world_model->ball.pos);
      });
    Point supporter_pos{0.0, 2.0};
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
          return world_model->getOurRobot(a)->getDistance(supporter_pos) <
                 world_model->getOurRobot(b)->getDistance(supporter_pos);
        }
      });

    kickoff_attack = std::make_shared<skills::KickoffAttack>(*best_attacker, world_model);
    attacker_command = std::make_shared<RobotCommandWrapper>(*best_attacker, world_model);
    kickoff_support = std::make_shared<skills::KickoffSupport>(*best_supporter, world_model);
    supporter_command = std::make_shared<RobotCommandWrapper>(*best_supporter, world_model);

    return {*best_attacker, *best_supporter};
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__OUR_KICKOFF_PLANNER_HPP_
