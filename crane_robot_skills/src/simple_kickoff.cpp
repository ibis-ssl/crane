// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/simple_kickoff.hpp>

namespace crane::skills
{
void SimpleKickOff::initializeParameters()
{
  kick_skill.setParameter("chip_kick", true);
  kick_skill.setParameter("use_target_chip_distance", true);
  kick_skill.setParameter("target_chip_distance", 2.0);
}

Status SimpleKickOff::update()
{
  kick_skill.setParameter("target", world_model()->getTheirGoalCenter());
  return kick_skill.run();
}
}  // namespace crane::skills
