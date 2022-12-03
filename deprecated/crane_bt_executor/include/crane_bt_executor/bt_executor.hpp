// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
