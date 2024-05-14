// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__WAITER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__WAITER_PLANNER_HPP_

#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class WaiterPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit WaiterPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("waiter", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> robot_commands;
    for (auto robot_id : robots) {
      crane::RobotCommandWrapper target(robot_id.robot_id, world_model);
      target.stopHere();
      target.setVelocity(0., 0.);
      if (target.robot->vel.linear.norm() < 0.5) {
        target.stopEmergency();
      }
      robot_commands.emplace_back(target.getMsg());
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    auto selected = this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        // choose id smaller first
        return 15. - static_cast<double>(-robot->id);
      },
      prev_roles);
    for (auto robot : selected) {
      stop_poses.emplace(robot, world_model->getOurRobot(robot)->pose);
    }
    return selected;
  }

private:
  //  rclcpp::TimerBase::SharedPtr timer;
  std::unordered_map<uint8_t, Pose2D> stop_poses;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__WAITER_PLANNER_HPP_
