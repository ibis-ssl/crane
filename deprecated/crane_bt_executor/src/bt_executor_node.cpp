// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "crane_bt_executor/bt_executor_component.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTExecutorComponent>());
  rclcpp::shutdown();
  return 0;
}
