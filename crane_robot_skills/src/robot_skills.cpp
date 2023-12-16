// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/get_ball_contact.hpp>
#include <crane_robot_skills/idle.hpp>
#include <crane_robot_skills/move_to_geometry.hpp>
#include <crane_robot_skills/move_with_ball.hpp>
#include <crane_robot_skills/turn_around_point.hpp>

namespace crane
{
template class MoveToGeometry<Point>;
template class MoveToGeometry<Segment>;
}  // namespace crane

//#include <rclcpp_components/register_node_macro.hpp>
//RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
