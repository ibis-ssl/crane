// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__OUR_FREE_KICK_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__OUR_FREE_KICK_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/simple_attacker.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class OurDirectFreeKickPlanner : public PlannerBase
{
private:
  std::shared_ptr<skills::SimpleAttacker> kicker = nullptr;

  std::vector<std::shared_ptr<RobotCommandWrapper>> other_robots;

public:
  COMPOSITION_PUBLIC
  explicit OurDirectFreeKickPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("OurDirectFreeKickPlanner", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num,
    const std::vector<uint8_t> & selectable_robots) -> std::vector<uint8_t> override;
};
}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__OUR_FREE_KICK_PLANNER_HPP_
