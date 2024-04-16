// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/time.hpp>

#include "crane_local_planner/local_planner.hpp"

namespace crane
{
void LocalPlannerComponent::callbackRobotCommands(const crane_msgs::msg::RobotCommands & msg)
{
  if (!world_model->hasUpdated()) {
    return;
  }
  ScopedTimer process_timer(process_time_pub);

  crane_msgs::msg::RobotCommands commands = msg;
  for (auto & command : commands.robot_commands) {
    command.current_ball_x = world_model->ball.pos.x();
    command.current_ball_y = world_model->ball.pos.y();
    auto robot = world_model->getOurRobot(command.robot_id);
    command.current_pose.x = robot->pose.pos.x();
    command.current_pose.y = robot->pose.pos.y();
    command.current_pose.theta = robot->pose.theta;
    command.current_velocity.x = robot->vel.linear.x();
    command.current_velocity.y = robot->vel.linear.y();
    command.current_velocity.theta = robot->vel.omega;
  }

  auto pub_msg = calculate_control_target(commands);
  pub_msg.header.stamp = rclcpp::Clock().now();
  commands_pub->publish(pub_msg);
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::LocalPlannerComponent)
