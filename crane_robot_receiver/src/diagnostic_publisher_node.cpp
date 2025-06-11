// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_receiver/diagnostic_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::DiagnosticPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
