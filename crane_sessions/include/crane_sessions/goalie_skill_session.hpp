// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__GOALIE_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__GOALIE_SKILL_SESSION_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/goalie.hpp>
#include <crane_sessions/single_skill_session.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class GoalieSkillSession : public SingleSkillSession<skills::Goalie>
{
public:
  COMPOSITION_PUBLIC explicit GoalieSkillSession(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : SingleSkillSession("goalie_skill", world_model)
  {
  }

  bool isHardConstraint() const override { return true; }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // Refereeが指定したゴールキーパーIDを優先的に割り当てる
    return [referee_goalie_id =
              world_model->getOurGoalieId()](const std::shared_ptr<RobotInfo> & robot) {
      // レフェリーが指定したゴールキーパーIDの場合、コスト0（最高の適性）
      if (robot->id == referee_goalie_id) {
        return 0.0;
      }
      // その他のロボットは非常に高いコストを返して避ける
      return GOALIE_EXCLUSION_COST;
    };
  }

protected:
  auto createSkill(uint8_t robot_id) -> std::shared_ptr<skills::Goalie> override
  {
    return std::make_shared<skills::Goalie>("goalie", robot_id, world_model);
  }
};
}  // namespace crane
#endif  // CRANE_SESSIONS__GOALIE_SKILL_SESSION_HPP_
