// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/emplace_robot.hpp>

namespace crane::skills
{
Status EmplaceRobot::update()
{
  Point target_position;

  double offset_x = getParameter<double>("robot_interval") *
                    (getParameter<int>("total_robot_number") - 1) * 0.5 * (-1.0);
  target_position.x() =
    (offset_x + getParameter<double>("robot_interval") * getParameter<int>("current_robot_index"));

  double position_y_side = getParameter<bool>("emplace_line_positive") ? 1.0 : -1.0;
  target_position.y() = position_y_side * world_model()->field_size.y() * 0.5;

  command->setTargetPosition(target_position).setMaxVelocity(getParameter<double>("max_speed"));
  return Status::RUNNING;
}
}  // namespace crane::skills
