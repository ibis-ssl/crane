// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__PASSABLE_BALL_PLACEMENT_TACTIC_HPP_
#define CRANE_TACTICS__PASSABLE_BALL_PLACEMENT_TACTIC_HPP_

#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class PassableBallPlacementTactic : public TacticBase
{
public:
  COMPOSITION_PUBLIC
  explicit PassableBallPlacementTactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("passable_ball_placement", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override
  {
    if (ball_placement) {
      ball_placement->run();
      return {TacticBase::Status::RUNNING, {ball_placement->getRobotCommand()}};
    } else {
      return {TacticBase::Status::RUNNING, {}};
    }
  }

  std::shared_ptr<skills::SingleBallPlacement> ball_placement;
};

class PlacementTargetPlacerTactic : public TacticBase
{
public:
  COMPOSITION_PUBLIC
  explicit PlacementTargetPlacerTactic(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("placement_target_placer", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override
  {
    if (placer) {
      placer->lookAtBall();
      placer->setDribblerTargetPosition(target);
      return {TacticBase::Status::RUNNING, {placer->getMsg()}};
    } else {
      return {TacticBase::Status::RUNNING, {}};
    }
  }

  std::shared_ptr<PositionCommandWrapper> placer = nullptr;

  Point target;
};
}  // namespace crane
#endif  // CRANE_TACTICS__PASSABLE_BALL_PLACEMENT_TACTIC_HPP_
