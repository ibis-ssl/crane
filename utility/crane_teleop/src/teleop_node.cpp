// Copyright (c) 2019 SSL-Roots
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_teleop/joystick_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<joystick::JoystickComponent> joystick_node =
    std::make_shared<joystick::JoystickComponent>(options);
  exe.add_node(joystick_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
}
