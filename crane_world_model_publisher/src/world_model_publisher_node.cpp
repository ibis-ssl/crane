// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <glog/logging.h>

#include <rclcpp/rclcpp.hpp>

#include "crane_world_model_publisher/world_model_publisher.hpp"

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::WorldModelPublisherComponent>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
