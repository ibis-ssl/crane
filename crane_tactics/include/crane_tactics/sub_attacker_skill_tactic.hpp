// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__SUB_ATTACKER_SKILL_TACTIC_HPP_
#define CRANE_TACTICS__SUB_ATTACKER_SKILL_TACTIC_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/sub_attacker.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class SubAttackerSkillTactic : public TacticBase
{
public:
  std::shared_ptr<skills::SubAttacker> skill = nullptr;

  COMPOSITION_PUBLIC explicit SubAttackerSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : TacticBase("sub_attacker_skill", world_model)
  {
  }

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // 注意: SubAttacker用のDPPS最適位置は将来game_analyzerのmetricsで計算予定
    // 現時点では簡略版を使用し、metrics追加後に更新する
    // TODO: game_analyzerにSubAttacker用metricsが追加されたら、
    //       recommended_sub_attacker_position等を使用するように更新
    auto wm = world_model;
    return [wm](const std::shared_ptr<RobotInfo> & robot) {
      // ボールが自陣にある場合は高コスト（SubAttacker不要）
      if (wm->point_checker.isInOurHalf(wm->ball().pos)) {
        return 10000.0;
      }
      // 敵ゴール方向でボール付近のロボットを優先
      return robot->getSquareDistance(wm->ball().pos);
    };
  }

protected:
  void onRobotsChanged() override { skill.reset(); }
};
}  // namespace crane
#endif  // CRANE_TACTICS__SUB_ATTACKER_SKILL_TACTIC_HPP_
