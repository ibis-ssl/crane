// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_target_area_planner/target_area_planner.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<crane::TargetAreaPlanner>(options);
  exe.add_node(node);
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
