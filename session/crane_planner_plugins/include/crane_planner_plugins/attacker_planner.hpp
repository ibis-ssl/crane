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

      auto set_target = [&](auto & target_array, auto value) {
        if (not target_array.empty()) {
          target_array.front() = value;
        } else {
          target_array.emplace_back(value);
        }
      };


      // 経由ポイント

      Point intermediate_point = world_model->ball.pos + (world_model->ball.pos - world_model->getTheirGoalCenter()).normalized() * 0.2;

      // ボールと敵ゴールの延長線上にいないときは，中間ポイントを経由
      double dot = (robot->pose.pos - world_model->ball.pos).normalized().dot((world_model->ball.pos - world_model->getTheirGoalCenter()).normalized());
      std::cout << "dot: " << dot << std::endl;
      if(dot < 0.95){
        set_target(target.target_x, intermediate_point.x());
        set_target(target.target_y, intermediate_point.y());
      }else{
        set_target(target.target_x, world_model->ball.pos.x());
        set_target(target.target_y, world_model->ball.pos.y());
        target.dribble_power = 0.5;
        target.kick_power = 0.5;
        target.chip_enable = false;
      }

      set_target(target.target_theta, getAngle(world_model->getTheirGoalCenter()-world_model->ball.pos));

      bool is_in_defense = world_model->isEnemyDefenseArea(world_model->ball.pos);
      bool is_in_field = world_model->isFieldInside(world_model->ball.pos);

      if(not is_in_field){
        // stop here
        target.motion_mode_enable = false;
        set_target(target.target_x, 0.0);
        set_target(target.target_y, 0.0);
        set_target(target.target_theta, 0.0);
      }
      control_targets.emplace_back(target);
    }
    return control_targets;
  }

  double getRoleScore(std::shared_ptr<RobotInfo> robot) override
  {
    // ボールに近いほどスコアが高い
    return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall({true, robot->id}), 0.01);
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__ATTACKER_PLANNER_HPP_
