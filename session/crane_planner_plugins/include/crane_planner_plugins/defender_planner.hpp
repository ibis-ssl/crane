// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/position_assignments.hpp>
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
class DefenderPlanner : public PlannerBase
{
public:
  void construct(WorldModelWrapper::SharedPtr world_model) override
  {
    PlannerBase::construct("defender", world_model);
  }
  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    auto ball = world_model->ball.pos;
    const double OFFSET_X = 0.1;
    const double OFFSET_Y = 0.1;

    //
    // calc ball line
    //
    Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
    {
      // シュート判定
      auto goal_posts = world_model->getOurGoalPosts();
      Segment goal_line(goal_posts.first, goal_posts.second);
      std::vector<Point> intersections;
      bg::intersection(ball_line, goal_line, intersections);
      if (intersections.empty()) {
        // シュートがなければ通常の動き
        ball_line.first = world_model->goal;
        ball_line.second = ball;
      }
    }

    std::vector<Point> defense_points = getDefensePoints(robots.size(), ball_line);

    if (not defense_points.empty()) {
      std::vector<Point> robot_points;
      for (auto robot_id : robots) {
        robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
      }

      auto solution = getOptimalAssignments(robot_points, defense_points);

      std::vector<crane_msgs::msg::RobotCommand> control_targets;
      for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
        int index = std::distance(robots.begin(), robot_id);
        Point target_point = defense_points[index];

        crane::RobotCommandWrapper target(robot_id->robot_id, world_model);
        auto robot = world_model->getRobot(*robot_id);

        target.setTargetPosition(target_point);
        target.setTargetTheta(getAngle(world_model->ball.pos - target_point));

        control_targets.emplace_back(target.getMsg());
      }
      return control_targets;
    } else {
      std::vector<crane_msgs::msg::RobotCommand> control_targets;
      for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
        int index = std::distance(robots.begin(), robot_id);
        Point target_point = defense_points[index];

        crane::RobotCommandWrapper target(robot_id->robot_id, world_model);

        auto robot = world_model->getRobot(*robot_id);

        // Stop at same position
        target.stopHere();

        control_targets.emplace_back(target.getMsg());
      }
      return control_targets;
    }
  }

  // defense_pointを中心にrobot_num台のロボットをdefense_line上に等間隔に配置する
  std::vector<Point> getDefensePoints(const int robot_num, const Segment & ball_line) const
  {
    const double DEFENSE_INTERVAL = 0.3;
    std::vector<Point> defense_points;

    if (auto defense_patrameter = getDefenseLinePointParameter(ball_line)) {
      double upper_parameter, lower_parameter;

      auto add_parameter = [&](double parameter) -> bool {
        const double OFFSET_X = 0.1, OFFSET_Y = 0.1;
        auto [threshold1, threshold2, threshold3] =
          getDefenseLinePointParameterThresholds(OFFSET_X, OFFSET_Y);
        if (parameter < 0. || parameter > threshold3) {
          return false;
        } else {
          if (upper_parameter < parameter) {
            upper_parameter = parameter;
          }
          if (lower_parameter > parameter) {
            lower_parameter = parameter;
          }
          defense_points.push_back(getDefenseLinePoint(parameter));
          return true;
        }
      };
      //1台目
      upper_parameter = *defense_patrameter;
      lower_parameter = *defense_patrameter;
      add_parameter(*defense_patrameter);

      // 2台目以降
      for (int i = 0; i < robot_num - 1; i++) {
        if (i % 2 == 0) {
          // upper側に追加
          if (not add_parameter(upper_parameter + DEFENSE_INTERVAL)) {
            // だめならlower側
            add_parameter(lower_parameter - DEFENSE_INTERVAL);
          }
        } else {
          // lower側に追加
          if (not add_parameter(lower_parameter - DEFENSE_INTERVAL)) {
            // だめならupper側
            add_parameter(upper_parameter + DEFENSE_INTERVAL);
          }
        }
      }
    }

    return defense_points;
  }

  auto getDefenseLinePoint(double parameter) const -> Point
  {
    const double OFFSET_X = 0.1, OFFSET_Y = 0.1;
    auto [p1, p2, p3, p4] = getDefenseAreaCorners(OFFSET_X, OFFSET_Y);

    const auto [threshold1, threshold2, threshold3] =
      getDefenseLinePointParameterThresholds(OFFSET_X, OFFSET_Y);

    if (parameter >= 0. && parameter < threshold1) {
      return p1 + (p2 - p1).normalized() * parameter;
    } else if (parameter < threshold2) {
      return p2 + (p3 - p2).normalized() * (parameter - threshold1);
    } else if (parameter < threshold3) {
      return p3 + (p4 - p3).normalized() * (parameter - threshold2);
    } else {
      throw std::runtime_error("Invalid parameter range for DefenderPlanner::getDefenseLinePoint");
    }
  }

  auto getDefenseLinePointParameterThresholds(double offset_x, double offset_y) const
    -> std::tuple<double, double, double>
  {
    const double threshold1 = world_model->defense_area_size.x() + offset_x;
    // p2 -> p3: world_model->defense_area_size.y() + OFFSET_Y * 2
    const double threshold2 = world_model->defense_area_size.y() + offset_y * 2 + threshold1;
    // p3 -> p4: world_model->defense_area_size.x() + OFFSET_X
    const double threshold3 = world_model->defense_area_size.x() + offset_x + threshold2;
    return {threshold1, threshold2, threshold3};
  }

  auto getDefenseAreaCorners(double offset_x, double offset_y) const
    -> std::tuple<Point, Point, Point, Point>
  {
    // デフェンスエリアを囲みし4つの点
    Point p1, p2, p3, p4;
    p1 << world_model->goal.x(), world_model->defense_area_size.y() * 0.5 + offset_y;
    p2 = p1;
    if (world_model->goal.x() > 0) {
      p2.x() -= (world_model->defense_area_size.x() + offset_x);
    } else {
      p2.x() += (world_model->defense_area_size.x() + offset_x);
    }
    p3 << p2.x(), -p2.y();
    p4 << p1.x(), p3.y();
    return {p1, p2, p3, p4};
  }

  auto getDefenseLinePointParameter(Segment target_segment) const -> std::optional<double>
  {
    const double OFFSET_X = 0.1, OFFSET_Y = 0.1;
    auto [p1, p2, p3, p4] = getDefenseAreaCorners(OFFSET_X, OFFSET_Y);

    const double threshold1 = world_model->defense_area_size.x() + OFFSET_X;
    // p2 -> p3: world_model->defense_area_size.y() + OFFSET_Y * 2
    const double threshold2 = world_model->defense_area_size.y() + OFFSET_Y * 2 + threshold1;

    std::vector<Point> intersections;

    if (bg::intersection(Segment{p1, p2}, target_segment, intersections);
        not intersections.empty()) {
      return std::abs(intersections[0].x() - p1.x());
    } else if (bg::intersection(Segment{p2, p3}, target_segment, intersections);
               not intersections.empty()) {
      return std::abs(intersections[0].y() - p2.y()) + threshold1;
    } else if (bg::intersection(Segment{p3, p4}, target_segment, intersections);
               not intersections.empty()) {
      return std::abs(intersections[0].x() - p3.x()) + threshold2;
    } else {
      //      throw std::runtime_error("Invalid target_segment for DefenderPlanner::getDefenseLinePointParameter");
      return std::nullopt;
    }
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
        // x座標が自ゴールに近いほうが優先
        return 20. - std::abs(world_model->goal.x() - robot->pose.pos.x());
      });
  }
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_
