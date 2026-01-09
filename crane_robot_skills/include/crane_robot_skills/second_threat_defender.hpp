// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SECOND_THREAT_DEFENDER_HPP_
#define CRANE_ROBOT_SKILLS__SECOND_THREAT_DEFENDER_HPP_

#include <crane_robot_skills/skill_base.hpp>
#include <string>
#include <utility>

namespace crane::skills
{
class SecondThreatDefender : public SkillBase
{
public:
  template <typename... Args>
  explicit SecondThreatDefender(Args &&... args)
  : SkillBase("SecondThreatDefender", std::forward<Args>(args)...)
  {
    initialize();
  }

  void initialize();

  static auto getDefaultPoint(const WorldModelWrapper::SharedPtr & world_model, const double offset)
    -> Point
  {
    // ボールと反対側にあるゴールの角
    return {
      (world_model->fieldSize().x() * 0.5 - world_model->getDefenseHeight() - offset) *
        world_model->getOurSideSign(),
      (world_model->getDefenseWidth() * 0.5 + offset) *
        ((world_model->ball().pos.y() > 0.) ? -1. : 1.)};
  }

  Status update() override;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SECOND_THREAT_DEFENDER_HPP_
