// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/position_assignments.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class DefenderPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit DefenderPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("defender", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  // defense_pointを中心にrobot_num台のロボットをdefense_line上に等間隔に配置する
  std::vector<Point> getDefensePoints(const int robot_num, const Segment & ball_line) const
  {
    const double DEFENSE_INTERVAL = 0.2;
    std::vector<Point> defense_points;

    if (auto defense_parameter = getDefenseLinePointParameter(ball_line)) {
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
      // 1台目
      upper_parameter = *defense_parameter;
      lower_parameter = *defense_parameter;
      add_parameter(*defense_parameter);

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
    auto [p1, p2, p3, p4] = getPenaltyAreaCorners(OFFSET_X, OFFSET_Y);

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
    const double threshold1 = world_model->penalty_area_size.x() + offset_x;
    // p2 -> p3: world_model->penalty_area_size.y() + OFFSET_Y * 2
    const double threshold2 = world_model->penalty_area_size.y() + offset_y * 2 + threshold1;
    // p3 -> p4: world_model->penalty_area_size.x() + OFFSET_X
    const double threshold3 = world_model->penalty_area_size.x() + offset_x + threshold2;
    return {threshold1, threshold2, threshold3};
  }

  auto getPenaltyAreaCorners(double offset_x, double offset_y) const
    -> std::tuple<Point, Point, Point, Point>
  {
    // デフェンスエリアを囲みし4つの点
    Point p1, p2, p3, p4;
    p1 << world_model->goal.x(), world_model->penalty_area_size.y() * 0.5 + offset_y;
    p2 = p1;
    if (world_model->goal.x() > 0) {
      p2.x() -= (world_model->penalty_area_size.x() + offset_x);
    } else {
      p2.x() += (world_model->penalty_area_size.x() + offset_x);
    }
    p3 << p2.x(), -p2.y();
    p4 << p1.x(), p3.y();
    return {p1, p2, p3, p4};
  }

  auto getDefenseLinePointParameter(Segment target_segment) const -> std::optional<double>
  {
    const double OFFSET_X = 0.1, OFFSET_Y = 0.1;
    auto [p1, p2, p3, p4] = getPenaltyAreaCorners(OFFSET_X, OFFSET_Y);

    const double threshold1 = world_model->penalty_area_size.x() + OFFSET_X;
    // p2 -> p3: world_model->penalty_area_size.y() + OFFSET_Y * 2
    const double threshold2 = world_model->penalty_area_size.y() + OFFSET_Y * 2 + threshold1;

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
      return std::nullopt;
    }
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    Segment ball_line{world_model->goal, world_model->ball.pos};
    auto parameter = getDefenseLinePointParameter(ball_line);
    if (not parameter) {
      return {};
    }
    const auto defense_point = getDefenseLinePoint(parameter.value());
    auto selected = this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [this, defense_point](const std::shared_ptr<RobotInfo> & robot) {
        // defense pointに近いほどスコアが高い
        return 100. - world_model->getSquareDistanceFromRobot(robot->id, defense_point);
      },
      prev_roles);

    return selected;
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__DEFENDER_PLANNER_HPP_
