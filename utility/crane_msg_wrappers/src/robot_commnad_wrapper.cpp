// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/robot_command_wrapper.hpp"

namespace crane
{
RobotCommandWrapperSimpleVelocity::RobotCommandWrapperSimpleVelocity(
  RobotCommandWrapperBase::SharedPtr & base)
: RobotCommandWrapperCommon(base)
{
  reset();
}

RobotCommandWrapperSimpleVelocity::RobotCommandWrapperSimpleVelocity(
  std::string skill_name, uint8_t id, WorldModelWrapper::SharedPtr world_model_wrapper)
: RobotCommandWrapperCommon(skill_name, id, world_model_wrapper)
{
  reset();
}

auto RobotCommandWrapperSimpleVelocity::reset() -> void
{
  command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
  command->latest_msg.local_camera_mode.clear();
  command->latest_msg.position_target_mode.clear();
  command->latest_msg.simple_velocity_target_mode.clear();
  command->latest_msg.velocity_target_mode.clear();
  command->latest_msg.simple_velocity_target_mode.emplace_back();

  x_controller.setGain(50.0, 0.0, 0.1);
  y_controller.setGain(50.0, 0.0, 0.1);
}

auto RobotCommandWrapperSimpleVelocity::setTargetPosition(
  Point target) -> RobotCommandWrapperSimpleVelocity &
{
  command->latest_msg.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
  if (command->latest_msg.simple_velocity_target_mode.empty()) {
    command->latest_msg.simple_velocity_target_mode.emplace_back();
  }
  const auto pos = getRobot()->pose.pos;
  Velocity vel;
  vel << x_controller.update(target.x() - pos.x(), 1. / 30.),
    y_controller.update(target.y() - pos.y(), 1. / 30.);
  if (vel.norm() > command->latest_msg.local_planner_config.max_velocity) {
    vel = vel.normalized() * command->latest_msg.local_planner_config.max_velocity;
  }
  command->latest_msg.simple_velocity_target_mode.front().target_vx = vel.x();
  command->latest_msg.simple_velocity_target_mode.front().target_vy = vel.y();
  return *this;
}
}  // namespace crane
