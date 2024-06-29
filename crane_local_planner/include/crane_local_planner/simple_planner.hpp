// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__SIMPLE_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__SIMPLE_PLANNER_HPP_

#include <crane_basics/pid_controller.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>

namespace crane
{
class SimplePlanner
{
public:
  explicit SimplePlanner(rclcpp::Node & node);

  crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model);

private:
  std::array<PIDController, 20> vx_controllers;
  std::array<PIDController, 20> vy_controllers;

  double MAX_VEL = 4.0;
  double P_GAIN = 4.0;
  double I_GAIN = 0.0;
  double D_GAIN = 0.0;

  rclcpp::Logger logger;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__SIMPLE_PLANNER_HPP_
