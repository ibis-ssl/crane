// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__KICK_OFF_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__KICK_OFF_PLANNER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/control_target.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "crane_planner_plugins/visibility_control.h"

namespace crane
{
class KickOffPlanner : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit KickOffPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("kickoff_planner", options), PlannerBase("kickoff", *this)
  {
    RCLCPP_INFO(get_logger(), "initializing");
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> control_targets;
    for (auto robot_id : robots) {
      crane_msgs::msg::RobotCommand target;
      auto robot = world_model->getRobot(robot_id);
      target.current_pose.x = robot->pose.pos.x();
      target.current_pose.y = robot->pose.pos.y();
      target.current_pose.theta = robot->pose.theta;
      // Stop at same position
      target.robot_id = robot_id.robot_id;
      target.chip_enable = false;
      target.dribble_power = 0.0;
      target.kick_power = 0.0;

      control_targets.emplace_back(target);
    }
    return control_targets;
  }
  double getRoleScore(std::shared_ptr<RobotInfo> robot) override
  {
    // the nearest to the ball first
    return 100. / world_model->getSquareDistanceFromRobotToBall({true, robot->id});
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__KICK_OFF_PLANNER_HPP_
