// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/marker.hpp>

namespace crane::skills
{
Marker::Marker(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase("Marker", id, wm)
{
  setParameter("marking_robot_id", 0);
  setParameter("mark_distance", 0.5);
  setParameter("mark_mode", std::string("save_goal"));
}

Status Marker::update([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  auto marked_robot = world_model->getTheirRobot(getParameter<int>("marking_robot_id"));
  auto enemy_pos = marked_robot->pose.pos;

  std::string mode = getParameter<std::string>("mark_mode");
  Point marking_point;
  double target_theta;

  if (mode == "save_goal") {
    marking_point = enemy_pos + (world_model->getOurGoalCenter() - enemy_pos).normalized() *
                                  getParameter<double>("mark_distance");
    target_theta = getAngle(enemy_pos - world_model->getOurGoalCenter());
  } else if (mode == "intercept_pass") {
    marking_point = enemy_pos + (world_model->ball.pos - enemy_pos).normalized() *
                                  getParameter<double>("mark_distance");
    target_theta = getAngle(enemy_pos - world_model->ball.pos);
  } else {
    throw std::runtime_error("unknown mark mode");
  }
  command->setTargetPosition(marking_point, target_theta);
  return Status::RUNNING;
}
}  // namespace crane::skills
