// Copyright (c) 2022 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
#include "rclcpp/rclcpp.hpp"

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
          rclcpp::get_logger(service_name.c_str()), "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger(service_name.c_str()), "service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger(service_name.c_str()), "service connected!");
  }

  std::shared_future<crane_msgs::srv::RobotSelect::Response::SharedPtr> sendRequest(
    crane_msgs::srv::RobotSelect::Request::SharedPtr request)
  {
    std::cout << "Sending request to " << name << std::endl;
    return client->async_send_request(request);
    // Wait for the result.

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
    request("replace", {1, 2, 3, 4});
  }

  void request(std::string situation, std::vector<int> selectable_robot_ids);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  WorldModelWrapper::SharedPtr world_model_;
  std::deque<crane_msgs::srv::RobotSelect::Request> query_queue_;
  rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr robot_select_client_;
  std::unordered_map<std::string, SessionModule::SharedPtr> session_planners_;
  //  identifier :  situation name,  content :   [ list of  [ pair of session name & selectable robot num]]
  std::unordered_map<std::string, std::vector<SessionCapacity>>
    robot_selection_priority_map;
};

}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
