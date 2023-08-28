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
    std::string service_name = "/session/" + name + "/robot_select";
    client_callback_group =
      node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client = node.create_client<crane_msgs::srv::RobotSelect>(
      service_name, rmw_qos_profile_services_default, client_callback_group);
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

  std::optional<crane_msgs::srv::RobotSelect::Response> sendRequest(
    crane_msgs::srv::RobotSelect::Request::SharedPtr request, rclcpp::Node & node)
  {
    std::cout << "Sending request to " << name << std::endl;
    auto result_future = client->async_send_request(request);
    std::cout << "Waiting for response from " << name << std::endl;

    using namespace std::chrono_literals;
    std::future_status status = result_future.wait_for(10s);
    if (status == std::future_status::ready) {
      return *result_future.get();
    } else {
      std::cout << "Failed to get response from " << name << std::endl;
      return std::nullopt;
    }

    //    // Wait for the result.
    //    if (
    //      rclcpp::spin_until_future_complete(node.get_node_base_interface(), result_future) ==
    //      rclcpp::FutureReturnCode::SUCCESS) {
    //      std::cout << "Received response from " << name << std::endl;
    //      return *result_future.get();
    //    } else {
    //      std::cout << "Failed to get response from " << name << std::endl;
    //      return std::nullopt;
    //    }
  }

  void clear()
  {
    auto request = std::make_shared<crane_msgs::srv::RobotSelect::Request>();
    request->selectable_robots_num = 0;
    auto result_future = client->async_send_request(request);
  }

private:
  rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr client;

  rclcpp::CallbackGroup::SharedPtr client_callback_group;

  const std::string name;
};
}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_MODULE_HPP_
