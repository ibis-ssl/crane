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
  const double OFFSET = getParameter<double>("offset");
  // ボールと反対側にあるゴールの角
  Point default_position{
    (world_model()->field_size.x() * 0.5 - world_model()->getDefenseHeight() - OFFSET) *
      world_model()->getOurSideSign(),
    (world_model()->getDefenseWidth() * 0.5 + OFFSET) *
      ((world_model()->ball.pos.y() > 0.) ? -1. : 1.)};
  command->setTargetPosition(default_position).lookAtBall();
  return Status::RUNNING;
}
}  // namespace crane::skills
