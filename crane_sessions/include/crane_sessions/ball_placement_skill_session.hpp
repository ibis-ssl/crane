// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__BALL_PLACEMENT_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__BALL_PLACEMENT_SKILL_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/single_ball_placement.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class BallPlacementSkillSession : public SessionBase
{
public:
  std::shared_ptr<skills::SingleBallPlacement> skill = nullptr;

  COMPOSITION_PUBLIC explicit BallPlacementSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SessionBase("ball_placement_skill", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      if (robot->id == wm->getOurGoalieId()) {
        return GOALIE_EXCLUSION_COST;  // ゴールキーパーは除外
      }
      // 「一番近いロボット」を直感通りに選ぶため線形距離を使用
      return robot->getDistance(wm->ball().pos);
    };
  }

protected:
  void onRobotsChanged() override { skill.reset(); }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__BALL_PLACEMENT_SKILL_SESSION_HPP_
