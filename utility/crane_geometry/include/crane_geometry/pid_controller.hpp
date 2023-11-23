// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__PID_CONTROLLER_HPP_
#define CRANE_GEOMETRY__PID_CONTROLLER_HPP_

namespace crane
{
class PIDController
{
public:
  PIDController() = default;
  
  void setGain(double p, double i, double d)
  {
    kp = p;
    ki = i;
    kd = d;
    error_prev = 0.0f;
  }

  double update(double error, double dt)
  {
    double p = kp * error;
    double i = ki * (error + error_prev) * dt / 2.0f;
    double d = kd * (error - error_prev) / dt;
    error_prev = error;
    return p + i + d;
  }

private:
  double kp, ki, kd;
  
  double error_prev;
};
}
#endif  // CRANE_GEOMETRY__PID_CONTROLLER_HPP_
