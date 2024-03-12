// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__TEMPLATE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__TEMPLATE_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class TemplatePlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit TemplatePlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("template", world_model, visualizer)
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
    uint8_t selectable_robots_num,
    const std::vector<uint8_t> & selectable_robots) -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
        // choose id smaller first
        return 15. - static_cast<double>(-robot->id);
      });
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__TEMPLATE_PLANNER_HPP_
