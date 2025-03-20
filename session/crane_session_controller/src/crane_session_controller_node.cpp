// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <glog/logging.h>

#include <memory>

#include "crane_session_controller/session_controller.hpp"

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::SessionControllerComponent>());
  rclcpp::shutdown();
  return 0;
}
