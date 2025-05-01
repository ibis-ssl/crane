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

  Status update() override;

  void print(std::ostream & os) const override { os << "[SecondThreatDefender]"; }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SECOND_THREAT_DEFENDER_HPP_
