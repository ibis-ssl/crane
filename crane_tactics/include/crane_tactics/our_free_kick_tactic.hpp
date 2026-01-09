// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__OUR_FREE_KICK_TACTIC_HPP_
#define CRANE_TACTICS__OUR_FREE_KICK_TACTIC_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class OurDirectFreeKickTactic : public TacticBase
{
private:
  std::shared_ptr<PositionCommandWrapper> kicker = nullptr;

  std::vector<std::shared_ptr<PositionCommandWrapper>> other_robots;

  bool fake_over = false;

  int fake_count = 0;

  // 前回のロボットロール情報を保存（calculatePositionCommand()で使用）
  std::unordered_map<uint8_t, RobotRole> cached_prev_roles;

public:
  COMPOSITION_PUBLIC
  explicit OurDirectFreeKickTactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("OurDirectFreeKickTactic", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override;
};
}  // namespace crane
#endif  // CRANE_TACTICS__OUR_FREE_KICK_TACTIC_HPP_
