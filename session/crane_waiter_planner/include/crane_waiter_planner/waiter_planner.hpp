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

#ifndef CRANE_WAITER_PLANNER__WAITER_PLANNER_HPP_
#define CRANE_WAITER_PLANNER__WAITER_PLANNER_HPP_

#include <functional>
#include <memory>

#include "crane_geometry/eigen_adapter.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/pass_info.hpp"
#include "crane_msgs/msg/receiver_plan.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msgs/srv/pass_request.hpp"
#include "crane_waiter_planner/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class WaiterPlanner : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit WaiterPlanner(const rclcpp::NodeOptions & options)
  : rclcpp::Node("waiter_planner", options)
  {
    world_model_ = std::make_shared<WorldModelWrapper>(*this);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  WorldModelWrapper::SharedPtr world_model_;
};

}  // namespace crane
#endif  // CRANE_WAITER_PLANNER__WAITER_PLANNER_HPP_
