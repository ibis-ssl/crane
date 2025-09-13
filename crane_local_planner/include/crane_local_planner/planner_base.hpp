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
  : visualizer(std::make_shared<VisualizerMessageBuilder>("local_planner/" + name))
  {
    world_model = std::make_shared<WorldModelWrapper>(node);
  }
  virtual auto calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg, double theta_offset)
    -> crane_msgs::msg::RobotCommands = 0;

  VisualizerMessageBuilder::SharedPtr visualizer;

  WorldModelWrapper::SharedPtr world_model;

  static auto resolveMaxAccelerationFactors(
    crane_msgs::msg::RobotCommand & command, const float default_max_acceleration) -> double
  {
    crane_msgs::msg::NamedFloat minimum_max_acceleration_factor;
    minimum_max_acceleration_factor.set__name("default").set__value(default_max_acceleration);
    for (const auto & factor : command.local_planner_config.max_acceleration_factors) {
      if (minimum_max_acceleration_factor.value > factor.value) {
        minimum_max_acceleration_factor = factor;
      }
    }
    command.local_planner_config.final_planned_max_acceleration = minimum_max_acceleration_factor;
    return command.local_planner_config.final_planned_max_acceleration.value;
  }

  static auto resolveMaxVelocityFactors(
    crane_msgs::msg::RobotCommand & command, const float default_max_velocity) -> double
  {
    crane_msgs::msg::NamedFloat minimum_max_velocity_factor;
    minimum_max_velocity_factor.set__name("default").set__value(default_max_velocity);
    for (const auto & factor : command.local_planner_config.max_velocity_factors) {
      if (minimum_max_velocity_factor.value > factor.value) {
        minimum_max_velocity_factor = factor;
      }
    }
    command.local_planner_config.final_planned_max_velocity = minimum_max_velocity_factor;
    return command.local_planner_config.final_planned_max_velocity.value;
  }
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_
