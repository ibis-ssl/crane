// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_local_planner/local_planner.hpp"

namespace crane
{
void LocalPlannerComponent::callbackRobotCommands(
  const crane_msgs::msg::RobotCommands & msg)
{
  if (!world_model->hasUpdated()) {
    return;
  }
  commands_pub->publish(calculate_control_target(msg));
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::LocalPlannerComponent)
