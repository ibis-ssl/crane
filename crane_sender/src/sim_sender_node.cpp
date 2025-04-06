// Copyright (c) 2019 SSL-Roots
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "crane_sender/sim_sender.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::SimSenderComponent>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
