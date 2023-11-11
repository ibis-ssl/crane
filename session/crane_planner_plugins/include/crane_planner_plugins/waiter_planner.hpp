// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__WAITER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__WAITER_PLANNER_HPP_

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
      crane::RobotCommandWrapper target(robot_id.robot_id, world_model);
      //      target.stopHere();
      target.setVelocity(0., 0.);
      target.setTargetTheta(target.robot->pose.theta);
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

private:
  //  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__WAITER_PLANNER_HPP_
