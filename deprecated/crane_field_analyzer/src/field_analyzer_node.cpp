// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "crane_field_analyzer/field_analyzer_component.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<crane::FieldAnalyzerComponent> field_analyzer_node =
      std::make_shared<crane::FieldAnalyzerComponent>(options);
  exe.add_node(field_analyzer_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
}
