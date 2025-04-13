// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__TOTAL_DEFENSE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__TOTAL_DEFENSE_PLANNER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/stream.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_planner_plugins/defense_functions.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <functional>
#include <memory>
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

  std::vector<std::shared_ptr<RobotCommandWrapperPosition>> first_threat_defenders;

  std::vector<std::shared_ptr<RobotCommandWrapperPosition>> second_threat_defenders;

private:
  bool m_is_goalie_total_defense_mode = true;

public:
  COMPOSITION_PUBLIC
  explicit TotalDefensePlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : PlannerBase("total_defense", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext & context) override;

  std::vector<Point> getDefenseArcPoints(const int robot_num, const Segment & ball_line) const;

  // defense_pointを中心にrobot_num台のロボットをdefense_line上に等間隔に配置する
  std::vector<Point> getDefenseLinePoints(const int robot_num, const Segment & ball_line, bool is_open_center, const double defense_parameter ) const;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext & context)
    -> std::vector<uint8_t> override;

private:
  Point getGoalieDefensePoint(const Segment& ball_line) const;
  std::vector<Point> getDefenseLinePoints(const int robot_num, const double defense_parameter, const bool is_open_center) const;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__TOTAL_DEFENSE_PLANNER_HPP_
