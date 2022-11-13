// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>

#include "crane_play_switcher/play_switcher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<crane::PlaySwitcher> play_switcher_node =
    std::make_shared<crane::PlaySwitcher>(options);
  exe.add_node(play_switcher_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
}
