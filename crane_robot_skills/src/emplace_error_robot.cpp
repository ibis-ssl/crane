// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/emplace_error_robot.hpp>

namespace crane::skills
{
Status EmplaceErrorRobot::update()
{
  Point target_position;
  target_position = Point(0.0, 0.0);
  command->setTargetPosition(target_position)
    .setMaxVelocity("EmplaceErrorRobot::max_speed", getParameter<double>("max_speed"));
  return Status::RUNNING;
}
}  // namespace crane::skills