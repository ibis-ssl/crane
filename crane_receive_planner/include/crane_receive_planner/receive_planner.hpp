// Copyright (c) 2021 ibis-ssl
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

#ifndef CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_
#define CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "crane_receive_planner/visibility_control.h"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/receiver_plan.hpp"
#include "crane_geometry/eigen_adapter.hpp"

namespace crane
{
class ReceivePlanner : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit ReceivePlanner(const rclcpp::NodeOptions & options) : rclcpp::Node("dummy_pass_planner",options){
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(1s,std::bind(&ReceivePlanner::timerCallback, this));
  }

  void timerCallback(){

  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
  std::shared_ptr<WorldModelWrapper> world_model_;
//  rclcpp::Publisher

  void world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    world_model_->update(*msg);
  }


};

}  // namespace crane
#endif  // CRANE_RECEIVE_PLANNER__RECEIVE_PLANNER_HPP_
