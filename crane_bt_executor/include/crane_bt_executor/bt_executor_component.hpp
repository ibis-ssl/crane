// Copyright (c) 2020 ibis-ssl
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

#ifndef CRANE_BT_EXECUTOR__BT_EXECUTOR_COMPONENT_HPP_
#define CRANE_BT_EXECUTOR__BT_EXECUTOR_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

class BTExecutorNode : public rclcpp::Node {
public:
  explicit BTExecutorNode(std::vector<std::string> plugin_name) : Node("bt_executor_node")
  {
    world_model_sub = create_subscription<crane_msgs::msg::WorldModel>(
        "/world_model", 10,
        std::bind(&BTExecutorNode::callbackWorldModel, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [/world_model] SET UP!");
    std::stringstream ss;
    ss << "/robot" << std::to_string(robot_id) << "/bt_cmd";
    bt_cmd_sub = create_subscription<crane_msgs::msg::BehaviorTreeCommand>(
        ss.str(), 10,
        std::bind(&BTExecutorNode::callbackBTCommand, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [%s] SET UP!", ss.str().c_str());

    timer = create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&BTExecutorNode::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "TIMER SET UP!");
  }
};
#endif
