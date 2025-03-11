// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/emplace_robot.hpp>

namespace crane::skills
{
EmplaceRobot::EmplaceRobot(RobotCommandWrapperBase::SharedPtr & base)
: SkillBase("EmplaceRobot", base)
{
  // このロボットのインデックス
  setParameter("current_robot_index", 0);
  setParameter("total_robot_number", 1);

  // yが+の位置に整列
  setParameter("emplace_line_positive", true);

  // 整列距離
  setParameter("robot_interval", 0.3);
  // ボールとの距離
  setParameter("margin_distance", 0.5);
  setParameter("max_speed", 1.5);
}

Status EmplaceRobot::update()
{
  Point target_position;

  double offset_x = getParameter<double>("robot_interval") *
                    (getParameter<int>("total_robot_number") - 1) * 0.5 * (-1.0);
  target_position.x() =
    (offset_x + getParameter<double>("robot_interval") * getParameter<int>("current_robot_index"));

  double position_y_side = getParameter<bool>("emplace_line_positive") ? 1.0 : -1.0;
  target_position.y() = position_y_side * world_model()->field_size.y() * 0.5;

  command.setTargetPosition(target_position).setMaxVelocity(getParameter<double>("max_speed"));
  return Status::RUNNING;
}
}  // namespace crane::skills
