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

#ifndef CRANE_GOALIE_PLANNER__GOALIE_PLANNER_HPP_
#define CRANE_GOALIE_PLANNER__GOALIE_PLANNER_HPP_

#include <functional>
#include <memory>

#include "crane_planner_plugins/visibility_control.h"
#include "crane_geometry/boost_geometry.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/control_target.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class GoaliePlanner : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit GoaliePlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("goalie_planner", options), PlannerBase("goalie", *this)
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

      auto ball = world_model_->ball.pos;
      std::cout << ball.x() << " " << ball.y() << ", "
                << static_cast<float>(world_model_->field_size.x()) << std::endl;

      Point goal_l, goal_r;
      goal_l << -world_model_->field_size.x() * 0.5f, 0.5f;
      goal_r << -world_model_->field_size.x() * 0.5f, -0.5f;
      Segment goal_line(goal_r, goal_l);

      Point goal_center;
      goal_center << -world_model_->field_size.x() * 0.5f, 0.0f;
      Segment ball_line(ball, ball + world_model_->ball.vel.normalized() * 20.f);
      std::vector<Point> intersections;

      // check shoot
      bg::intersection(ball_line, goal_line, intersections);
      if (not intersections.empty()) {
        std::cout << "Shoot block mode" << std::endl;
        ClosestPoint result;
        bg::closest_point(ball_line, robot->pose.pos, result);
        // position control
        target.motion_mode_enable = false;
        target.target.x = result.closest_point.x();
        target.target.y = result.closest_point.y();
      } else {
        // go blocking point
        std::cout << "Normal blocking mode" << std::endl;
        const double BLOCK_DIST = 1.0;
        target.motion_mode_enable = false;
        Point target_point;
        target_point = goal_center + (ball - goal_center).normalized() * BLOCK_DIST;
        target.target.x = target_point.x();
        target.target.y = target_point.y();
      }

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
};

}  // namespace crane
#endif  // CRANE_GOALIE_PLANNER__GOALIE_PLANNER_HPP_
