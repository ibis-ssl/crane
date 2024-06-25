// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__PID_CONTROLLER_HPP_
#define CRANE_BASICS__PID_CONTROLLER_HPP_

#include <algorithm>

namespace crane
{
class PIDController
{
public:
  PIDController() = default;

  void setGain(double p, double i, double d, double max_int = -1.0)
  {
    kp = p;
    ki = i;
    kd = d;
    error_prev = 0.0f;
    max_integral = max_int;
  }

  double update(double error, double dt)
  {
    double p = kp * error;
    integral += error * dt;
    if (max_integral > 0.0) {
      integral = std::clamp(integral, -max_integral, max_integral);
    }
    double d = kd * (error - error_prev) / dt;
    error_prev = error;
    return p + integral * ki + d;
  }

private:
  double kp, ki, kd;

  double error_prev;

  double integral = 0.0;

  double max_integral = -1.0;
};
}  // namespace crane
#endif  // CRANE_BASICS__PID_CONTROLLER_HPP_
