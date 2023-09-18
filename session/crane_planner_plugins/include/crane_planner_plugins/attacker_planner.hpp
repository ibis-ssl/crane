// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_

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
class AttackerPlanner : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit AttackerPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("attacker_planner", options), PlannerBase("attacker", *this)
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

      target.robot_id = robot_id.robot_id;
      target.chip_enable = false;
      target.dribble_power = 0.0;
      target.kick_power = 0.0;
      target.motion_mode_enable = false;

      // 経由ポイント

      Point intermediate_point =
        world_model->ball.pos +
        (world_model->ball.pos - world_model->getTheirGoalCenter()).normalized() * 0.2;


      double dot = (robot->pose.pos - world_model->ball.pos)
                     .normalized()
                     .dot((world_model->ball.pos - world_model->getTheirGoalCenter()).normalized());
      double target_theta = getAngle(world_model->getTheirGoalCenter() - world_model->ball.pos);
      // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
      if (dot < 0.95 || std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.05) {
        setTarget(target.target_x, intermediate_point.x());
        setTarget(target.target_y, intermediate_point.y());
        target.local_planner_config.disable_collision_avoidance = false;
      } else {
        setTarget(target.target_x, world_model->ball.pos.x());
        setTarget(target.target_y, world_model->ball.pos.y());
        target.dribble_power = 0.5;
        target.kick_power = 0.5;
        target.chip_enable = false;
        target.local_planner_config.disable_collision_avoidance = true;
      }

      setTarget(
        target.target_theta, getAngle(world_model->getTheirGoalCenter() - world_model->ball.pos));

      bool is_in_defense = world_model->isEnemyDefenseArea(world_model->ball.pos);
      bool is_in_field = world_model->isFieldInside(world_model->ball.pos);

      if (not is_in_field) {
        // stop here
        target.motion_mode_enable = false;
        setTarget(target.target_x, world_model->goal.x()/2.);
        setTarget(target.target_y, world_model->goal.y()/2.);
        setTarget(target.target_theta, getAngle(-world_model->goal));
      }
      control_targets.emplace_back(target);
    }
    return control_targets;
  }

protected:
  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
      // ボールに近いほどスコアが高い
      return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall({true, robot->id}), 0.01);
    });
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_
