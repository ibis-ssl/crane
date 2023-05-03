// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_FIELD_ANALYZER__ROLES__DEFENDER_HPP_
#define CRANE_FIELD_ANALYZER__ROLES__DEFENDER_HPP_

#include "crane_field_analyzer/roles/role_base.hpp"

namespace crane
{
class DefenderRole : public RoleBase
{
public:
  DefenderRole() : RoleBase() {}
  bool isAvailable(const crane_msgs::msg::PlaySituation & msg) const override;
  void calcRoleScore(
    const PlaySituationWrapper & play_situation, RoleScoreWrapper & role_score) override;
  void calcGoalieScore(const WorldModelWrapper & world_model);
  void calcFirstThreatDefenderScore(const WorldModelWrapper & world_model, ) void getSuccessRate(
    const WorldModel & world_model, geometry2d::Point target, std::shared_ptr<RobotNode> R);
};

bool DefenderRole::isAvailable(const crane_msgs::msg::PlaySituation & msg) const { return false; }
void DefenderRole::calcRoleScore(
  const PlaySituationWrapper & play_situation, RoleScoreWrapper & role_score)
{
}

}  // namespace crane
#endif  // CRANE_FIELD_ANALYZER__ROLES__DEFENDER_HPP_
