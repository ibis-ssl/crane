// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SIMPLE_KICKOFF_HPP_
#define CRANE_ROBOT_SKILLS__SIMPLE_KICKOFF_HPP_

#include <algorithm>
#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/kick.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace crane::skills
{
class SimpleKickOff : public SkillBase
{
public:
  template <typename... Args>
  explicit SimpleKickOff(Args &&... args)
  : SkillBase("SimpleKickOff", std::forward<Args>(args)...), kick_skill(command)
  {
    initializeParameters();
  }

  void initializeParameters();

  Status update() override;

  Kick kick_skill;

private:
  bool kicked = false;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SIMPLE_KICKOFF_HPP_
