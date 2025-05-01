// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/second_threat_defender.hpp>

namespace crane::skills
{
void SecondThreatDefender::initialize() { setParameter("offset", 0.3); }

Status SecondThreatDefender::update()
{
  auto default_position = getDefaultPoint(world_model(), getParameter<double>("offset"));
  command->setTargetPosition(default_position).lookAtBall();
  return Status::RUNNING;
}
}  // namespace crane::skills
