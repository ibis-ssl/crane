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

#ifndef CRANE_TEMPLATE_PLANNER__TEMPLATE_PLANNER_HPP_
#define CRANE_TEMPLATE_PLANNER__TEMPLATE_PLANNER_HPP_

#include <functional>
#include <memory>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/control_target.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "crane_planner_plugins/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class TemplatePlanner : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit TemplatePlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("template_planner", options), PlannerBase("template", *this)
  {
    RCLCPP_INFO(get_logger(), "initializing");
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> control_targets;
    for (auto robot_id : robots) {
      crane_msgs::msg::RobotCommand target;
      auto robot = world_model_->getRobot(robot_id);

      target.robot_id = robot_id.robot_id;
      target.chip_enable = false;
      target.dribble_power = 0.0;
      target.kick_power = 0.0;

      // TODO: implement
      target.motion_mode_enable = false;
      target.target.x = 0.0;
      target.target.y = 0.0;
      target.target.theta = 0.0;  // omega

      control_targets.emplace_back(target);
    }
    return control_targets;
  }
  double getRoleScore(std::shared_ptr<RobotInfo> robot) override
  {
    // TODO: implement
  }
};

}  // namespace crane
#endif  // CRANE_TEMPLATE_PLANNER__TEMPLATE_PLANNER_HPP_
