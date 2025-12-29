// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__TOTAL_DEFENSE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__TOTAL_DEFENSE_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_planner_plugins/defense_functions.hpp>
#include <crane_planner_plugins/marker_functions.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <crane_robot_skills/marker.hpp>
#include <crane_robot_skills/second_threat_defender.hpp>
#include <crane_utils/stream.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class TotalDefensePlanner : public PlannerBase
{
public:
  std::shared_ptr<skills::Goalie> goalie;

  std::shared_ptr<skills::SecondThreatDefender> second_threat_defender;

  std::vector<std::shared_ptr<skills::Marker>> markers;

private:
  bool m_is_goalie_total_defense_mode = true;

  std::mutex markers_mutex;

  /// マーキングターゲットを割り当て
  auto assignMarkingTargets(const std::vector<uint8_t> & available_robots) -> std::vector<uint8_t>;

public:
  COMPOSITION_PUBLIC
  explicit TotalDefensePlanner(
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node & node)
  : PlannerBase("total_defense", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__TOTAL_DEFENSE_PLANNER_HPP_
