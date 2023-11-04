// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_planner_plugins/tigers_goalie_planner.hpp"

namespace crane
{
}
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::TigersGoaliePlanner)

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(crane::TigersGoaliePlanner, crane::PlannerBase)
