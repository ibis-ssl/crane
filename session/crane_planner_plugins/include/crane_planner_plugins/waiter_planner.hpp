// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__WAITER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__WAITER_PLANNER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/control_target.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "crane_planner_plugins/visibility_control.h"

namespace crane
{
class WaiterPlanner : public PlannerBase
{
public:
  void construct(WorldModelWrapper::SharedPtr world_model) override
  {
    PlannerBase::construct("waiter", world_model);
  }
  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> control_targets;
    for (auto robot_id : robots) {
      crane_msgs::msg::RobotCommand target;
      auto robot = world_model->getRobot(robot_id);
      // Stop at same position
      target.robot_id = robot_id.robot_id;
      target.chip_enable = false;
      target.dribble_power = 0.0;
      target.kick_power = 0.0;
      // control by velocity
      target.motion_mode_enable = true;

      // 強制的にゼロにする
      target.target_velocity.x = 0.0;
      target.target_velocity.y = 0.0;
      target.target_velocity.theta = 0.0;

      // 位置目標を削除
      target.target_x.clear();
      target.target_y.clear();
      target.target_theta.clear();

      control_targets.emplace_back(target);
    }
    return control_targets;
  }
  double getRoleScore(std::shared_ptr<RobotInfo> robot) override
  {
    // choose id smaller first
    return 15. - static_cast<double>(-robot->id);
  }

private:
  //  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__WAITER_PLANNER_HPP_
