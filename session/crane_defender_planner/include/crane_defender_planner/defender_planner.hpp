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

#ifndef CRANE_DEFENDER_PLANNER__DEFENDER_PLANNER_HPP_
#define CRANE_DEFENDER_PLANNER__DEFENDER_PLANNER_HPP_

#include <functional>
#include <memory>

#include "crane_defender_planner/visibility_control.h"
#include "crane_geometry/boost_geometry.hpp"
#include "crane_geometry/position_assignments.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/control_target.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class DefenderPlannerComponent : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit DefenderPlannerComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("defender_planner", options), PlannerBase("defender", *this)
  {
    RCLCPP_INFO(get_logger(), "initializing");
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::cout << "calculating control target" << std::endl;
    auto ball = world_model_->ball.pos;
    Point area_left_bottom, area_left_top, area_right_bottom, area_right_top;
    const double OFFEST_X = 0.5;
    const double OFFEST_Y = 0.5;
    area_left_bottom << -world_model_->field_size.x() * 0.5,
      world_model_->defense_area.y() * 0.5 + OFFEST_Y;
    area_left_top = area_left_bottom;
    area_left_top.x() += world_model_->defense_area.x() + OFFEST_X;

    area_right_bottom << -world_model_->field_size.x() * 0.5,
      -world_model_->defense_area.y() * 0.5 - OFFEST_Y;
    area_right_top = area_left_bottom;
    area_right_top.x() += world_model_->defense_area.x() + OFFEST_X;

    Point goal_center(-world_model_->field_size.x() * 0.5, 0.0);

    std::vector<Segment> segments;
    segments.emplace_back(area_left_bottom, area_left_top);
    segments.emplace_back(area_right_bottom, area_right_top);
    segments.emplace_back(area_left_top, area_right_top);

    //
    // calc ball line
    //
    Segment ball_line(ball, ball + world_model_->ball.vel.normalized() * 20.f);
    {
      Point goal_l, goal_r;
      goal_l << -world_model_->field_size.x() * 0.5f, 0.5f;
      goal_r << -world_model_->field_size.x() * 0.5f, -0.5f;
      Segment goal_line(goal_r, goal_l);
      std::vector<Point> intersections;
      bg::intersection(ball_line, goal_line, intersections);
      if (intersections.empty()) {
        ball_line.first = goal_center;
        ball_line.second = ball;
      }
    }

    //
    // calculate defense_point
    //
    std::vector<Point> intersections;
    Point defense_point;
    Segment defense_line;
    for (auto seg : segments) {
      bg::intersection(seg, ball_line, intersections);
      if (not intersections.empty()) {
        defense_point = intersections.front();
        defense_line = seg;
        break;
      }
    }

    std::cout << "end calculate defense_point " << std::endl;

    //
    // list up defense points
    //
    const double DEFENSE_INTERVAL = 0.5;
    std::vector<Point> defense_points;
    Point diff_defense_line = defense_line.first - defense_line.second;
    if (diff_defense_line.x() = 0.) {
      // front defense
      if (robots.size() % 2 == 0) {
        defense_point.y() -= DEFENSE_INTERVAL * 0.5;
      }
      defense_points.push_back(defense_point);
      for (int i = 0; i < robots.size() - 1; i++) {
        Point p = defense_point;
        if (i % 2 == 0) {
          p.y() += DEFENSE_INTERVAL * (i / 2 + 1);
        } else {
          p.y() -= DEFENSE_INTERVAL * (i / 2 + 1);
        }
        defense_points.push_back(p);
      }
    } else {
      // side defense
      if (robots.size() % 2 == 0) {
        defense_point.x() -= DEFENSE_INTERVAL * 0.5;
      }
      defense_points.push_back(defense_point);
      for (int i = 0; i < robots.size() - 1; i++) {
        Point p = defense_point;
        if (i % 2 == 0) {
          p.x() += DEFENSE_INTERVAL * (i / 2 + 1);
        } else {
          p.x() -= DEFENSE_INTERVAL * (i / 2 + 1);
        }
        defense_points.push_back(p);
      }
    }

    std::cout << "end list up defense_point " << std::endl;
    std::vector<Point> robot_points;
    for (auto robot_id : robots) {
      robot_points.emplace_back(world_model_->getRobot(robot_id)->pose.pos);
    }

    std::cout << "solve start" << std::endl;
    auto solution = getOptimalAssignments(robot_points, defense_points);

    std::cout << "solve end" << std::endl;
    std::vector<crane_msgs::msg::RobotCommand> control_targets;
    for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
      int index = std::distance(robots.begin(), robot_id);
      Point target_point = defense_points[index];

      crane_msgs::msg::RobotCommand target;
      auto robot = world_model_->getRobot(*robot_id);
      // Stop at same position
      target.robot_id = robot_id->robot_id;
      target.chip_enable = false;
      target.dribble_power = 0.0;
      target.kick_power = 0.0;
      // control by position
      target.motion_mode_enable = false;
      // Stop at same position
      target.target.x = target_point.x();  // vx
      target.target.y = target_point.y();  // vy
      target.target.theta = 0.0;           // omega
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
#endif  // CRANE_DEFENDER_PLANNER__DEFENDER_PLANNER_HPP_
