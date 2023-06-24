// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__SESSION_MODULE_HPP_
#define CRANE_SESSION_CONTROLLER__SESSION_MODULE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_msgs/srv/robot_select.hpp"

namespace crane
{
class SessionModule
{
public:
  using SharedPtr = std::shared_ptr<SessionModule>;

  SessionModule(std::string name) : name(name) {}

  void construct(rclcpp::Node & node)
  {
    node_base = node.get_node_base_interface();
    std::string service_name = "/session/" + name + "/assign";
    client = node.create_client<crane_msgs::srv::RobotSelect>(service_name);
    using namespace std::chrono_literals;
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(service_name.c_str()),
          "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO(
        rclcpp::get_logger(service_name.c_str()), "service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger(service_name.c_str()), "service connected!");
  }

  std::optional<crane_msgs::srv::RobotSelect::Response> assign(
    crane_msgs::srv::RobotSelect::Request::SharedPtr request)
  {
    std::cout << "Assigning robots to " << name << std::endl;
    auto result_future = client->async_send_request(request);
    // Wait for the result.
    if (
      rclcpp::spin_until_future_complete(node_base, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      std::cout << "Received response from " << name << std::endl;
      return *result_future.get();
    } else {
      std::cout << "Failed to get response from " << name << std::endl;
      return std::nullopt;
    }
  }

  void observe(){}

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base = nullptr;

  rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr client;

  const std::string name;

  std::vector<int> assigned_robots;
};
}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_MODULE_HPP_
