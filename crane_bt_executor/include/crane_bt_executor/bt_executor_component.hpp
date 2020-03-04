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
#include <crane_msgs/msg/world_model.hpp>

class BTExecutorComponent : public rclcpp::Node {
public:
  BTExecutorComponent() : Node("bt_executor_node")
  {
    world_model_sub_ = create_subscription<crane_msgs::msg::WorldModel>(
        "/crane_world_observer/world_model", 10,
        std::bind(&BTExecutorComponent::callbackWorldModel, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "SUBSCRIBER [/world_model] SET UP!");

    timer_ = create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&BTExecutorComponent::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "TIMER SET UP!");
  }

  void callbackWorldModel(crane_msgs::msg::WorldModel::ConstSharedPtr msg){}
  void timerCallback(){}
private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif  // CRANE_BT_EXECUTOR__BT_EXECUTOR_COMPONENT_HPP_

