// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "kickoff_planner.hpp"

namespace crane
{
}
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::KickOffPlanner)

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(crane::KickOffPlanner, crane::PlannerBase)
