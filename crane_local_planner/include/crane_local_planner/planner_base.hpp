// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_
#define CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_

#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>

namespace crane
{
class LocalPlannerBase
{
public:
  LocalPlannerBase(std::string name, rclcpp::Node & node)
  {
    world_model = std::make_shared<WorldModelWrapper>(node);
    visualizer = std::make_shared<ConsaiVisualizerWrapper>(node, name);
  }
  virtual crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg) = 0;

  ConsaiVisualizerWrapper::SharedPtr visualizer;

protected:
  WorldModelWrapper::SharedPtr world_model;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_
