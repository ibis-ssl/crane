// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__BALL_NEAR_BY_POSITIONER_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__BALL_NEAR_BY_POSITIONER_SKILL_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/ball_nearby_positioner.hpp>
#include <crane_sessions/session_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class BallNearByPositionerSkillSession : public SessionBase
{
public:
  std::vector<std::shared_ptr<skills::BallNearByPositioner>> skills;

  COMPOSITION_PUBLIC explicit BallNearByPositionerSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &,
    const std::string & session_name = "ball_nearby_positioner_skill")
  : SessionBase(session_name, world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;
    return [wm, exclude_goalie = shouldExcludeGoalie()](const std::shared_ptr<RobotInfo> & robot) {
      if (exclude_goalie && robot->id == wm->getOurGoalieId()) {
        return GOALIE_EXCLUSION_COST;  // ゴールキーパーは除外
      }
      return robot->getSquareDistance(wm->ball().pos);
    };
  }

protected:
  void onRobotsChanged() override { skills.clear(); }

  /// ゴールキーパーを除外するか（サブクラスでオーバーライド可能）
  virtual bool shouldExcludeGoalie() const { return true; }

  /// positioning_policyパラメータ値（サブクラスでオーバーライド可能）
  virtual std::string getPositioningPolicy() const { return "auto"; }

  /// スキル実行前のカスタム設定（サブクラスでオーバーライド可能）
  virtual void setupBeforeRun(
    [[maybe_unused]] const std::shared_ptr<skills::BallNearByPositioner> & skill)
  {
  }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__BALL_NEAR_BY_POSITIONER_SKILL_SESSION_HPP_
