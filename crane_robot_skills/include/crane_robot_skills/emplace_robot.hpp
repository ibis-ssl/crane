// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__EMPLACE_ROBOT_HPP_
#define CRANE_ROBOT_SKILLS__EMPLACE_ROBOT_HPP_

#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class EmplaceRobot : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit EmplaceRobot(RobotCommandWrapperBase::SharedPtr & base);

  Status update() override;

  void print(std::ostream & os) const override { os << "[EmplaceRobot]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__EMPLACE_ROBOT_HPP_