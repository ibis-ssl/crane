// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__OUR_KICKOFF_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__OUR_KICKOFF_PLANNER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/kickoff_attack.hpp>
#include <crane_robot_skills/kickoff_support.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class OurKickOffPlanner : public PlannerBase
{
private:
  std::shared_ptr<skills::KickoffAttack> kickoff_attack;

  std::shared_ptr<skills::KickoffSupport> kickoff_support;

public:
  COMPOSITION_PUBLIC explicit OurKickOffPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("our_kickoff_planner", world_model, visualizer)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__OUR_KICKOFF_PLANNER_HPP_
