// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__OUR_PENALTY_KICK_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__OUR_PENALTY_KICK_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/penalty_kick.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class OurPenaltyKickPlanner : public PlannerBase
{
private:
  std::shared_ptr<skills::PenaltyKick> skill = nullptr;
  std::shared_ptr<RobotCommandWrapper> robot_command_wrapper = nullptr;

public:
  COMPOSITION_PUBLIC
  explicit OurPenaltyKickPlanner(
    WorldModelWrapper::SharedPtr & world_model, ConsaiVisualizerWrapper::SharedPtr visualizer)
  : PlannerBase("OurPenaltyKickPlanner", world_model, visualizer)
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

      // TODO(HansRobo): implement
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
    auto robots_sorted = this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [&](const std::shared_ptr<RobotInfo> & robot) {
        // ボールに近いほうが先頭
        return 100. / robot->getDistance(world_model->ball.pos);
      });
    if (robots_sorted.size() > 0) {
      // 一番ボールに近いロボットがキッカー
    }
    if (robots_sorted.size() > 1) {
      // 2番目以降はボールより1m以上下がる(ルール5.3.5.3)
    }
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__OUR_PENALTY_KICK_PLANNER_HPP_
