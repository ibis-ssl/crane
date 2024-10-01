// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__SIMPLE_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__SIMPLE_PLANNER_HPP_

#include <algorithm>
#include <crane_basics/pid_controller.hpp>
#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planner_base.hpp"

namespace crane
{
class SimplePlanner : public LocalPlannerBase
{
public:
  explicit SimplePlanner(rclcpp::Node & node) : LocalPlannerBase("simple_local_planner", node)
  {
    node.declare_parameter("max_vel", MAX_VEL);
    MAX_VEL = node.get_parameter("max_vel").as_double();
  }

  crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg) override
  {
    crane_msgs::msg::RobotCommands commands = msg;
    for (auto & command : commands.robot_commands) {
      auto robot = world_model->getOurRobot(command.robot_id);
      if (not command.position_target_mode.empty()) {
        visualizer->addLine(
          robot->pose.pos.x(), robot->pose.pos.y(), command.position_target_mode.front().target_x,
          command.position_target_mode.front().target_y, 1);
      }
      if (command.local_planner_config.max_velocity > MAX_VEL) {
        command.local_planner_config.max_velocity = MAX_VEL;
      }
    }
    visualizer->publish();
    return commands;
  }

private:
  double MAX_VEL = 4.0;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__SIMPLE_PLANNER_HPP_
