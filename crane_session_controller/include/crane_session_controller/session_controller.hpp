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
  SessionModule(std::string name, rclcpp::Node::SharedPtr node) : name(name), node(node) {}
  void construct()
  {
    client = node->create_client<crane_msgs::srv::RobotSelect>("robot_select");
    using namespace std::chrono_literals;
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          rclcpp::get_logger(name.c_str()), "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger(name.c_str()), "service not available, waiting again...");
    }
  }

  std::optional<crane_msgs::srv::RobotSelect::Response> sendRequest(
    crane_msgs::srv::RobotSelect::Request::SharedPtr request)
  {
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      return result.get();
    } else {
      return nullopt;
    }
  }

private:
  rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr client;
  const std::string name;
  const rclcpp::Node::SharedPtr node;
};

class SessionContoller : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit SessionContoller(const rclcpp::NodeOptions & options)
  : rclcpp::Node("session_controller", options), replace_planner_("replace")
  {
    replace_planner_.construct(this);

    timer_ = create_wall_timer(1s, std::bind(&SessionContoller::timerCallback, this));
  }
  void timerCallback() {}

  void request() {}

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
  std::shared_ptr<WorldModelWrapper> world_model_;
  std::deque<crane_msgs::srv::RobotSelect::Request> query_queue_;
  rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr robot_select_client_;
  SessionModule replace_planner_;
//  situation > planner ,
  std::unordered_map<std::string, >

  void world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    world_model_->update(*msg);
  }
};

}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
