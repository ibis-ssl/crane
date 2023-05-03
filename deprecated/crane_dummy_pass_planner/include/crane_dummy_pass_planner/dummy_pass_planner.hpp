// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_DUMMY_PASS_PLANNER__DUMMY_PASS_PLANNER_HPP_
#define CRANE_DUMMY_PASS_PLANNER__DUMMY_PASS_PLANNER_HPP_

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_dummy_pass_planner/visibility_control.h"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/pass_plan.hpp"
#include "crane_msgs/msg/world_model.hpp"

namespace crane
{
class DummyPassPlanner : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit DummyPassPlanner(const rclcpp::NodeOptions & options)
  : rclcpp::Node("dummy_pass_planner", options)
  {
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1s, std::bind(&DummyPassPlanner::timerCallback, this));
  }
  void timerCallback() {}

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
  WorldModelWrapper::SharedPtr world_model_;

  void world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    world_model_->update(*msg);
  }
};

}  // namespace crane
#endif  // CRANE_DUMMY_PASS_PLANNER__DUMMY_PASS_PLANNER_HPP_
