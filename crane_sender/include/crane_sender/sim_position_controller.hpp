// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SIM_POSITION_CONTROLLER_HPP_
#define CRANE_SENDER__SIM_POSITION_CONTROLLER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msgs/msg/robot_command.hpp>

namespace crane
{

struct SimPositionControllerConfig
{
  double position_gain = 2.0;
  double deceleration = 3.0;
};

[[nodiscard]] auto rotateFieldVector(const Vector2 & vector, double theta_offset) -> Vector2;

[[nodiscard]] auto calculateSimGlobalVelocity(
  const crane_msgs::msg::RobotCommand & command, const SimPositionControllerConfig & config)
  -> Vector2;

}  // namespace crane

#endif  // CRANE_SENDER__SIM_POSITION_CONTROLLER_HPP_
