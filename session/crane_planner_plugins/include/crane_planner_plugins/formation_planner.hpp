// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__FORMATION_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__FORMATION_PLANNER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/position_assignments.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class FormationPlanner : public PlannerBase
{
public:
  enum class FormationType {
    WING,
    IBIS,
  };
  COMPOSITION_PUBLIC
  explicit FormationPlanner(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &, const FormationType formation_type)
  : PlannerBase("formation", world_model), formation_type(formation_type)
  {
  }

  std::vector<Point> getWingFormationPoints(int robot_num);

  std::vector<Point> getIbisFormationPoints(int robot_num);

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots, PlannerContext &) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles, PlannerContext &)
    -> std::vector<uint8_t> override;

  const FormationType formation_type;
};

class IbisFormationPlanner final : public FormationPlanner
{
public:
  COMPOSITION_PUBLIC
  explicit IbisFormationPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : FormationPlanner(world_model, node, FormationType::IBIS)
  {
  }
};

class WingFormationPlanner final : public FormationPlanner
{
public:
  COMPOSITION_PUBLIC
  explicit WingFormationPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : FormationPlanner(world_model, node, FormationType::WING)
  {
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__FORMATION_PLANNER_HPP_
