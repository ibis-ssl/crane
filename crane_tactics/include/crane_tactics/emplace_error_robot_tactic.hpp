// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__EMPLACE_ERROR_ROBOT_TACTIC_HPP_
#define CRANE_TACTICS__EMPLACE_ERROR_ROBOT_TACTIC_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <crane_robot_skills/emplace_error_robot.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class EmplaceErrorRobotTactic : public TacticBase
{
public:
  COMPOSITION_PUBLIC explicit EmplaceErrorRobotTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("EmplaceErrorRobot", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  bool isHardConstraint() const override
  {
    return false;
  };

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override;

private:
  std::unordered_map<uint8_t, std::shared_ptr<skills::EmplaceErrorRobot>> m_skill_map;
};


}  // namespace crane
#endif  // CRANE_TACTICS__EMPLACE_ERROR_ROBOT_TACTIC_HPP_
