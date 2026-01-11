// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__EMPLACE_ERROR_ROBOT_HPP_
#define CRANE_ROBOT_SKILLS__EMPLACE_ERROR_ROBOT_HPP_

#include <crane_robot_skills/skill_base.hpp>

namespace crane::skills
{
class EmplaceErrorRobot : public SkillBase
{
public:
  template <typename... Args>
  explicit EmplaceErrorRobot(Args &&... args) : SkillBase("EmplaceErrorRobot", std::forward<Args>(args)...)
  {
    /*
    *Robot0(master) \
    *                 ErrRobot (進行方向→)
    *Robot1(slave)  /
    */
    // このロボットのインデックス 0 master, 1 slave
    setParameter("current_robot_index", 0);
    setParameter("max_speed", 0.5);
    setParameter("emplace_line_positive", true);
  }

  Status update() override;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__EMPLACE_ERROR_ROBOT_HPP_