// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_sender/sim_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <crane_geometry/geometry_operations.hpp>

namespace crane
{

auto rotateFieldVector(const Vector2 & vector, double theta_offset) -> Vector2
{
  return rotate(vector, theta_offset);
}

auto calculateSimGlobalVelocity(
  const crane_msgs::msg::RobotCommand & command, const SimPositionControllerConfig & config)
  -> Vector2
{
  if (command.position_target_mode.empty()) {
    return Vector2::Zero();
  }
  const auto & target = command.position_target_mode.front();
  const Vector2 error(
    target.target_x - command.current_pose.x, target.target_y - command.current_pose.y);
  Vector2 feedforward(target.terminal_velocity_x, target.terminal_velocity_y);
  const double terminal_limit = std::max(0.0f, target.speed_limit_at_target);
  if (terminal_limit > 0.0 && feedforward.norm() > terminal_limit) {
    feedforward *= terminal_limit / feedforward.norm();
  }
  if (error.norm() <= target.position_tolerance && feedforward.norm() < 1e-4) {
    return Vector2::Zero();
  }

  Vector2 velocity = config.position_gain * error + feedforward;
  const double max_velocity =
    std::max(0.0f, command.local_planner_config.final_planned_max_velocity.value);
  const double terminal_speed = feedforward.norm();
  const double braking_limit = std::sqrt(
    terminal_speed * terminal_speed + 2.0 * std::max(0.0, config.deceleration) * error.norm());
  const double speed_limit = std::min(max_velocity, braking_limit);
  if (velocity.norm() > speed_limit && velocity.norm() > 1e-9) {
    velocity *= speed_limit / velocity.norm();
  }
  return rotateFieldVector(velocity, command.field_coordinate_theta_offset);
}

}  // namespace crane
