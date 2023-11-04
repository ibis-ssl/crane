// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "game_analyzer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<crane::GameAnalyzerComponent> game_analyzer_node =
    std::make_shared<crane::GameAnalyzerComponent>(options);
  exe.add_node(game_analyzer_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
}
