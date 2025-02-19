// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_
#define CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <memory>

namespace crane
{
class LocalPlannerBase
{
public:
  LocalPlannerBase(const std::string & name, rclcpp::Node & node)
  : visualizer(std::make_unique<CraneVisualizerBuffer::MessageBuilder>("local_planner/" + name))
  {
    world_model = std::make_shared<WorldModelWrapper>(node);
  }
  virtual crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg) = 0;

  CraneVisualizerBuffer::MessageBuilder::UniquePtr visualizer;

  WorldModelWrapper::SharedPtr world_model;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_
