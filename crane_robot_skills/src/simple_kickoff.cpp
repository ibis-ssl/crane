// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/simple_kickoff.hpp>

namespace crane::skills
{
SimpleKickOff::SimpleKickOff(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("SimpleKickOff", base), kick_skill(base)
{
  kick_skill.setParameter("kick_power", 1.0);
  kick_skill.setParameter("chip_kick", true);
}

Status SimpleKickOff::update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  kick_skill.setParameter("target", world_model()->getTheirGoalCenter());
  return kick_skill.run(visualizer);
}
}  // namespace crane::skills
