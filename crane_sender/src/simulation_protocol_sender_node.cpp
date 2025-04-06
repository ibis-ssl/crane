// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "crane_sender/simulation_protocol_sender.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::SimulationProtocolSenderComponent>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
