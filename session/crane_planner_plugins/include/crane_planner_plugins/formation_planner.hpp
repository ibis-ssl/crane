// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__FORMATION_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__FORMATION_PLANNER_HPP_

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
class FormationPlanner : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit FormationPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("formation_planner", options), PlannerBase("formation", *this)
  {
    RCLCPP_INFO(get_logger(), "initializing");
  }

  std::vector<Point> getFormationPoints(int robot_num)
  {
    std::vector<Point> formation_points;
    formation_points.emplace_back(0.5, 0.0);
    formation_points.emplace_back(0.8, 0.5);
    formation_points.emplace_back(0.8, -0.5);
    formation_points.emplace_back(1.5, 0.0);
    formation_points.emplace_back(1.5, 1.0);
    formation_points.emplace_back(1.5, -1.0);

    if(world_model->getOurGoalCenter().x() < 0.0){
      for(auto &point : formation_points){
        point.x() *= -1.0;
      }
    }

    formation_points.resize(robot_num);
    return formation_points;
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {

    std::vector<Point> robot_points;
    for (auto robot_id : robots) {
      robot_points.emplace_back(world_model->getRobot(robot_id)->pose.pos);
    }
    auto formation_points = getFormationPoints(robots.size());

    auto solution = getOptimalAssignments(robot_points, formation_points);

    double target_theta = (world_model->getOurGoalCenter().x() > 0.0)? M_PI : 0.0;
    std::vector<crane_msgs::msg::RobotCommand> control_targets;
    for (auto robot_id = robots.begin(); robot_id != robots.end(); ++robot_id) {
      int index = std::distance(robots.begin(), robot_id);
      Point target_point = formation_points[index];

      crane_msgs::msg::RobotCommand target;
      auto robot = world_model->getRobot(*robot_id);
      target.current_pose.x = robot->pose.pos.x();
      target.current_pose.y = robot->pose.pos.y();
      target.current_pose.theta = robot->pose.theta;

      // Stop at same position
      target.robot_id = robot_id->robot_id;
      target.chip_enable = false;
      target.dribble_power = 0.0;
      target.kick_power = 0.0;
      // control by position
      target.motion_mode_enable = false;

      // Stop at same position
      setTarget(target.target_x, target_point.x());
      setTarget(target.target_y, target_point.y());
      setTarget(target.target_theta, target_theta);

      control_targets.emplace_back(target);
    }
    return control_targets;
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
      // choose id smaller first
      return 15. - static_cast<double>(-robot->id);
    });
  }

private:
  //  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__FORMATION_PLANNER_HPP_
