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

#ifndef CRANE_BT_EXECUTOR__BT_EXECUTOR_HPP_
#define CRANE_BT_EXECUTOR__BT_EXECUTOR_HPP_

#include <chrono>
#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "crane_msgs/msg/behavior_tree_command.hpp"
#include "crane_msgs/msg/robot_command.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_world_observer/world_model.hpp"

class BTExecutor : public rclcpp::Node
{
public:
  BTExecutor(uint8_t robot_id, std::vector<std::string> plugin_names) : Node("bt_executor")
  {
  }

private:
  void timerCallback()
  {
    RCLCPP_INFO(this->get_logger(), "tick");
  }
  void callbackWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    world_model_.update(msg);
  }

private:
  int zmp_port_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub_;
  rclcpp::Subscription<crane_msgs::msg::BehaviorTreeCommand>::SharedPtr bt_cmd_sub_;
  crane_msgs::msg::RobotCommand robot_command_;
  WorldModelWrapper world_model_;
};
#endif  // CRANE_BT_EXECUTOR__BT_EXECUTOR_HPP_
