// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WAITER_PLANNER__WAITER_PLANNER_HPP_
#define CRANE_WAITER_PLANNER__WAITER_PLANNER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/control_target.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "crane_planner_plugins/visibility_control.h"

namespace crane
{
class WaiterPlanner : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit WaiterPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("waiter_planner", options), PlannerBase("waiter", *this)
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
      // Stop at same position
      target.robot_id = robot_id.robot_id;
      target.chip_enable = false;
      target.dribble_power = 0.0;
      target.kick_power = 0.0;
      // control by velocity
      target.motion_mode_enable = true;
      // Stop at same position
      target.target.x = 0.0;      // vx
      target.target.y = 0.0;      // vy
      target.target.theta = 0.0;  // omega
      control_targets.emplace_back(target);
    }
    return control_targets;
  }
  double getRoleScore(std::shared_ptr<RobotInfo> robot) override
  {
    // choose id smaller first
    return 15. - static_cast<double>(-robot->id);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace crane
#endif  // CRANE_WAITER_PLANNER__WAITER_PLANNER_HPP_
