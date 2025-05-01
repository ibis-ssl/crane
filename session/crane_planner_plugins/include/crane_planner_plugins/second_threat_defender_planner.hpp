// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__SECOND_THREAT_DEFENDER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__SECOND_THREAT_DEFENDER_PLANNER_HPP_

#include <algorithm>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/second_threat_defender.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class SecondThreatDefenderPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit SecondThreatDefenderPlanne(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : PlannerBase("second_threat_defender", world_model)
  {
  }

  auto calculateRobotCommand(const std::vector<RobotIdentifier> & robots, PlannerContext &)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override
  {
    if (skill) {
      return {PlannerBase::Status::RUNNING, {skill->getRobotCommand()}};
    } else {
      return {PlannerBase::Status::RUNNING, {}};
    }
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override
  {
    if (selectable_robots_num < 1) {
      return {};
    } else {
      constexpr double offset = 0.3;
      auto target = skills::SecondThreatDefender::getDefaultPoint(world_model, offset);
      auto selected = this->getSelectedRobotsByScore(
        1, selectable_robots,
        [this](const std::shared_ptr<RobotInfo> & robot) {
          // ターゲットに一番近いロボット
          return 100. / robot->getDistance(target);
        },
        prev_roles, context);
      skill = std::make_shared<skills::SecondThreatDefender>(selected.front(), world_model);
      skill->setParameter("offset", offset);
      return selected;
    }
  }

private:
  std::shared_ptr<skills::SecondThreatDefender> skill;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__SECOND_THREAT_DEFENDER_PLANNER_HPP_
