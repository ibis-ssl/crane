// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/move_to_geometry.hpp>

namespace crane
{
template class MoveToGeometry<Point>;
template class MoveToGeometry<Segment>;
}  // namespace crane

//#include <rclcpp_components/register_node_macro.hpp>
//RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
