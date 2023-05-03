// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "crane_rotation_planner/visibility_control.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  /*
  rclcpp::NodeOptions options:
  std::shared_ptr<crane::RotationPlanner> play_switcher_node =
    std::make_shared<crane::RotationPlanner>(options);
  exe.add_node(play_switcher_node->get_node_base_interface())
  exe.spin();
  rclcpp::shutdown()
  */
  return 0;
}
