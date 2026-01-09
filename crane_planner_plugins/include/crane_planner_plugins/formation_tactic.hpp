// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__FORMATION_TACTIC_HPP_
#define CRANE_PLANNER_PLUGINS__FORMATION_TACTIC_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_planner_plugins/tactic_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class FormationTactic : public TacticBase
{
public:
  enum class FormationType {
    WING,
    IBIS,
  };
  COMPOSITION_PUBLIC
  explicit FormationTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &, const FormationType formation_type)
  : TacticBase("formation", world_model), formation_type(formation_type)
  {
  }

  std::vector<Point> getWingFormationPoints(int robot_num);

  std::vector<Point> getIbisFormationPoints(int robot_num);

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;

  const FormationType formation_type;
};

class IbisFormationTactic final : public FormationTactic
{
public:
  COMPOSITION_PUBLIC
  explicit IbisFormationTactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : FormationTactic(world_model, node, FormationType::IBIS)
  {
  }
};

class WingFormationTactic final : public FormationTactic
{
public:
  COMPOSITION_PUBLIC
  explicit WingFormationTactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : FormationTactic(world_model, node, FormationType::WING)
  {
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__FORMATION_TACTIC_HPP_
