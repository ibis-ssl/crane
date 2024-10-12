// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__CUT_PASS_HPP_
#define CRANE_ROBOT_SKILLS__CUT_PASS_HPP_

#include <crane_basics/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

namespace crane::skills
{
class CutPass : public SkillBase<RobotCommandWrapperPosition>
{
public:
  explicit CutPass(RobotCommandWrapperBase::SharedPtr & base);

  Status update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) override;

  void print(std::ostream & os) const override { os << "[CutPass]"; }

  Point getInterceptionPoint() const;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__CUT_PASS_HPP_
