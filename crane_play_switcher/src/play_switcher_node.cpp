// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_play_switcher/play_switcher.hpp"

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::PlaySwitcher>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
}
