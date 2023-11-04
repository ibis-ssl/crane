// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__GOALIE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__GOALIE_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/control_target.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

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
      crane::RobotCommandWrapper target(robot_id.robot_id, world_model);
      auto robot = world_model->getRobot(robot_id);

      auto ball = world_model->ball.pos;

      auto goals = world_model->getOurGoalPosts();

      Segment goal_line(goals.first, goals.second);

      Point goal_center;
      goal_center << goals.first.x(), 0.0f;
      Segment ball_line(ball, ball + world_model->ball.vel * 20000000.f);
      std::vector<Point> intersections;

      // check shoot
      bg::intersection(ball_line, goal_line, intersections);
      Point target_point;
      float target_theta;
      if (not intersections.empty()) {
        //        std::cout << "Shoot block mode" << std::endl;
        ClosestPoint result;
        bg::closest_point(ball_line, robot->pose.pos, result);
        target_point << result.closest_point.x(), result.closest_point.y();
        // position control
        target_theta = getAngle(-world_model->ball.vel);
      } else {
        // go blocking point
        //        std::cout << "Normal blocking mode" << std::endl;
        const double BLOCK_DIST = 0.25;

        // 範囲外のときは正面に構える
        if (not world_model->isFieldInside(world_model->ball.pos)) {
          ball << 0, 0;
        }
        target_point = goal_center + (ball - goal_center).normalized() * BLOCK_DIST;
        target_theta = getAngle(ball - target_point);
      }

      target.setTargetPosition(target_point, target_theta);

      control_targets.emplace_back(target.getMsg());
    }
    return control_targets;
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
        // choose id smaller first
        return 15. - static_cast<double>(-robot->id);
      });
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__GOALIE_PLANNER_HPP_
