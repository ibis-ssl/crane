// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__PASSABLE_BALL_PLACEMENT_TACTIC_HPP_
#define CRANE_SESSIONS__PASSABLE_BALL_PLACEMENT_TACTIC_HPP_

#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class PassableBallPlacementSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC
  explicit PassableBallPlacementSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("passable_ball_placement", world_model)
  {
  }

  auto calculatePositionCommand([[maybe_unused]] const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override
  {
    if (ball_placement) {
      ball_placement->run();
      return {SessionBase::Status::RUNNING, {ball_placement->getRobotCommand()}};
    } else {
      return {SessionBase::Status::RUNNING, {}};
    }
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == wm->getOurGoalieId()) {
        return 10000.0;  // ゴールキーパーは除外
      }
      return robot->getSquareDistance(wm->ball().pos);
    };
  }

  std::shared_ptr<skills::SingleBallPlacement> ball_placement;
};

class PlacementTargetPlacerSession : public SessionBase
{
public:
  COMPOSITION_PUBLIC
  explicit PlacementTargetPlacerSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("placement_target_placer", world_model)
  {
  }

  auto calculatePositionCommand([[maybe_unused]] const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override
  {
    if (placer) {
      placer->lookAtBall();
      placer->setDribblerTargetPosition(target);
      return {SessionBase::Status::RUNNING, {placer->getMsg()}};
    } else {
      return {SessionBase::Status::RUNNING, {}};
    }
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == wm->getOurGoalieId()) {
        return 10000.0;  // ゴールキーパーは除外
      }
      if (auto target = wm->getBallPlacementTarget(); target) {
        return robot->getDistance(target.value());
      }
      return robot->getDistance(wm->ball().pos);  // フォールバック
    };
  }

  std::shared_ptr<PositionCommandWrapper> placer = nullptr;

  Point target;
};
}  // namespace crane
#endif  // CRANE_SESSIONS__PASSABLE_BALL_PLACEMENT_TACTIC_HPP_
