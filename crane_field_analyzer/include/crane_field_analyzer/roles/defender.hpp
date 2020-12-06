// Copyright (c) 2020 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_FIELD_ANALYZER__ROLES__DEFENDER_HPP_
#define CRANE_FIELD_ANALYZER__ROLES__DEFENDER_HPP_

#include "crane_field_analyzer/roles/role_base.hpp"

namespace crane
{
class DefenderRole : public RoleBase
{
public:
  DefenderRole()
  : RoleBase() {}
  bool isAvailable(const crane_msgs::msg::PlaySituation & msg) const override;
  void calcRoleScore(
    const PlaySituationWrapper & play_situation,
    RoleScoreWrapper & role_score) override;
  void calcGoalieScore(const WorldModelWrapper &world_model);
  void calcFirstThreatDefenderScore(const WorldModelWrapper &world_model, )
  void getSuccessRate(const WorldModel &world_model, geometry2d::Point target, std::shared_ptr<RobotNode> R);
};

bool DefenderRole::isAvailable(
  const crane_msgs::msg::PlaySituation & msg) const
{
  return false;
}
void DefenderRole::calcRoleScore(
  const PlaySituationWrapper & play_situation,
  RoleScoreWrapper & role_score) {

}


}
#endif  // CRANE_FIELD_ANALYZER__ROLES__DEFENDER_HPP_