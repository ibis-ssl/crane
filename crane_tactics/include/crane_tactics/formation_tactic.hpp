// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__FORMATION_TACTIC_HPP_
#define CRANE_TACTICS__FORMATION_TACTIC_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_tactics/tactic_base.hpp>
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
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &, const FormationType formation_type,
    const std::string & tactic_name = "formation")
  : TacticBase(tactic_name, world_model), formation_type(formation_type)
  {
  }

  std::vector<Point> getWingFormationPoints(int robot_num);

  std::vector<Point> getIbisFormationPoints(int robot_num);

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    return [](const std::shared_ptr<RobotInfo> & robot) {
      return static_cast<double>(robot->id);  // ID小優先
    };
  }

  const FormationType formation_type;
};

class IbisFormationTactic final : public FormationTactic
{
public:
  COMPOSITION_PUBLIC
  explicit IbisFormationTactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : FormationTactic(world_model, node, FormationType::IBIS, "ibis_formation")
  {
  }
};

class WingFormationTactic final : public FormationTactic
{
public:
  COMPOSITION_PUBLIC
  explicit WingFormationTactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : FormationTactic(world_model, node, FormationType::WING, "wing_formation")
  {
  }
};

}  // namespace crane
#endif  // CRANE_TACTICS__FORMATION_TACTIC_HPP_
