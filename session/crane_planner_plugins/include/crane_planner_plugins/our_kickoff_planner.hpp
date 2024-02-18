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

#include "visibility_control.h"

namespace crane
{
class OurKickOffPlanner : public PlannerBase
{
private:
  std::shared_ptr<skills::KickoffAttack> kickoff_attack_;

  std::shared_ptr<skills::KickoffSupport> kickoff_support_ public
  : COMPOSITION_PUBLIC explicit OurKickOffPlanner(
      WorldModelWrapper::SharedPtr & world_model, ConsaiVisualizerWrapper::SharedPtr visualizer)
  : PlannerBase("our_kickoff_planner", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto robot_id : robots) {
      crane_msgs::msg::RobotCommand target;
      auto robot = world_model->getRobot(robot_id);

      target.robot_id = robot_id.robot_id;
      target.chip_enable = false;
      target.dribble_power = 0.0;
      target.kick_power = 0.0;

      // TODO: implement
      target.motion_mode_enable = false;

      //      setTarget(target.target_x, 0.0);
      //      setTarget(target.target_y, 0.0);
      //      target.target_velocity.theta = 0.0;

      robot_commands.emplace_back(target);
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    // 一番ボールに近いロボットをkickoff attack
    auto best_attacker = std::max_element(
      selectable_robots.begin(), selectable_robots.end(),
      [this](const uint8_t & a, const uint8_t & b) {
        return world_model->getRobot(a)->getDistance(world_model->ball.pos) <
               world_model->getRobot(b)->getDistance(world_model->ball.pos);
      });
    Point supporter_pos{0.0, 2.0};
    auto best_supporter = std::max_element(
      selectable_robots.begin(), selectable_robots.end(),
      [this, supporter_pos](const uint8_t & a, const uint8_t & b) {
        return world_model->getRobot(a)->getDistance(supporter_pos) <
               world_model->getRobot(b)->getDistance(supporter_pos);
      });

    // TODO return
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__OUR_KICKOFF_PLANNER_HPP_
