// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/simple_kickoff.hpp>

namespace crane::skills
{
Status SimpleKickOff::update()
{
  if (!kicked && world_model()->ball().isMoving(0.5)) {
    kicked = true;
  }

  if (kicked) {
    command->stopHere();
    return Status::RUNNING;
  }

  return goal_kick_skill.run();
}
}  // namespace crane::skills
