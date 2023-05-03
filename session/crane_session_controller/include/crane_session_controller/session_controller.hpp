// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
#define CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_session_controller/visibility_control.h"
#include <rclcpp/rclcpp.hpp>

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

  std::optional<crane_msgs::srv::RobotSelect::Response> sendRequest(
    crane_msgs::srv::RobotSelect::Request::SharedPtr request, rclcpp::Node & node)
  {
    std::cout << "Sending request to " << name << std::endl;
    auto result_future = client->async_send_request(request);
    // Wait for the result.
    if (
      rclcpp::spin_until_future_complete(node.get_node_base_interface(), result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      std::cout << "Received response from " << name << std::endl;
      return *result_future.get();
    } else {
      std::cout << "Failed to get response from " << name << std::endl;
      return std::nullopt;
    }
  }

private:
  rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr client;
  const std::string name;
};

struct SessionCapacity
{
  std::string session_name;
  int selectable_robot_num;
};

class SessionControllerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit SessionControllerComponent(const rclcpp::NodeOptions & options);

  void timerCallback() {}

  void reassignRequestCallback() { testAssignRequest(); }

  void testAssignRequest()
  {
    // expect : {goalie : 1}, {replace : 2}, {waiter : 1}
    request("replace", {0, 1, 2, 3});
  }

  void request(std::string situation, std::vector<int> selectable_robot_ids);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  WorldModelWrapper::SharedPtr world_model_;
  std::deque<crane_msgs::srv::RobotSelect::Request> query_queue_;
  rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr robot_select_client_;
  std::unordered_map<std::string, SessionModule::SharedPtr> session_planners_;
  //  identifier :  situation name,  content :   [ list of  [ pair of session name & selectable robot num]]
  std::unordered_map<std::string, std::vector<SessionCapacity>> robot_selection_priority_map;
};

}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
