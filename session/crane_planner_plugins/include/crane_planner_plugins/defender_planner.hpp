// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_geometry/position_assignments.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/control_target.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "crane_planner_plugins/visibility_control.h"

namespace crane
{
class DefenderPlanner : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit DefenderPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("defender_planner", options), PlannerBase("defender", *this)
  {
    RCLCPP_INFO(get_logger(), "initializing");
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    auto ball = world_model->ball.pos;
    Point area_left_bottom, area_left_top, area_right_bottom, area_right_top;
    const double OFFEST_X = 0.1;
    const double OFFEST_Y = 0.1;
    area_left_bottom << -world_model->field_size.x() * 0.5,
      world_model->defense_area.y() * 0.5 + OFFEST_Y;
    area_left_top = area_left_bottom;
    area_left_top.x() += world_model->defense_area.x() + OFFEST_X;

    area_right_bottom << -world_model->field_size.x() * 0.5,
      -world_model->defense_area.y() * 0.5 - OFFEST_Y;
    area_right_top = area_right_bottom;
    area_right_top.x() += world_model->defense_area.x() + OFFEST_X;

    std::vector<Segment> segments;
    segments.emplace_back(area_left_bottom, area_left_top);
    segments.emplace_back(area_right_bottom, area_right_top);
    segments.emplace_back(area_left_top, area_right_top);

    //
    // calc ball line
    //
    Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
    {
      Point goal_l, goal_r;
      goal_l << -world_model->field_size.x() * 0.5f, 0.5f;
      goal_r << -world_model->field_size.x() * 0.5f, -0.5f;
      Segment goal_line(goal_r, goal_l);
      std::vector<Point> intersections;
      bg::intersection(ball_line, goal_line, intersections);
      if (intersections.empty()) {
        Point goal_center(-world_model->field_size.x() * 0.5, 0.0);
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

    //
    // list up defense points
    //
    std::vector<Point> defense_points =
      getDefensePoints(robots.size(), defense_point, defense_line);

    std::vector<Point> robot_points;
    for (auto robot_id : robots) {
      robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
    }

    auto solution = getOptimalAssignments(robot_points, defense_points);

    std::vector<crane_msgs::msg::RobotCommand> control_targets;
    for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
      int index = std::distance(robots.begin(), robot_id);
      Point target_point = defense_points[index];

      crane_msgs::msg::RobotCommand target;
      auto robot = world_model->getRobot(*robot_id);
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
  std::vector<Point> getDefensePoints(
    const int robot_num, Point defense_point, const Segment & defense_line) const
  {
    const double DEFENSE_INTERVAL = 0.3;
    std::vector<Point> defense_points;
    Point diff_defense_line = defense_line.first - defense_line.second;
    if (diff_defense_line.x() == 0.) {
      // front defense
      if (robot_num % 2 == 0) {
        defense_point.y() -= DEFENSE_INTERVAL * 0.5;
      }
      defense_points.push_back(defense_point);
      for (int i = 0; i < robot_num - 1; i++) {
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
      if (robot_num % 2 == 0) {
        defense_point.x() -= DEFENSE_INTERVAL * 0.5;
      }
      defense_points.push_back(defense_point);
      for (int i = 0; i < robot_num - 1; i++) {
        Point p = defense_point;
        if (i % 2 == 0) {
          p.x() += DEFENSE_INTERVAL * (i / 2 + 1);
        } else {
          p.x() -= DEFENSE_INTERVAL * (i / 2 + 1);
        }
        defense_points.push_back(p);
      }
    }
    return defense_points;
  }
  double getRoleScore(std::shared_ptr<RobotInfo> robot) override
  {
    // choose id smaller first
    return 20. - robot->pose.pos.x();
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_
