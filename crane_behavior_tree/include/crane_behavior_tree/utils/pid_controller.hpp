// Copyright (c) 2020 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_BEHAVIOR_TREE__UTILS__PID_CONTROLLER_HPP_
#define CRANE_BEHAVIOR_TREE__UTILS__PID_CONTROLLER_HPP_

#include <cmath>
#include <algorithm>

class PIDController
{
public:
  PIDController(
    const float kp, const float ki, const float kd, const float isat = 0.f,
    const float min = -1.f, float max = 1.f)
  : kp(kp), ki(ki), kd(kd), isat(isat), min(min), max(max)
  {
    reset();
  }

  virtual ~PIDController() {}

  void update(const float current, const float target)
  {
    static float e, ed;
    e = target - current;
    ed = e - ep;
    ei += e;
    ep = e;
    if (isat > 0.f) {ei = std::clamp(ei, -isat, isat);}   // Anti wind-up
    u = kp * e + ki * ei + kd * ed;
    u = std::clamp(u, min, max);
  }

  float getOutput() const {return u;}

  void reset()
  {
    ei = 0.f;
    ep = 0.f;
    u = 0.f;
  }

  void setGain(const float kp, const float ki, const float kd)
  {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
  }

  void setGain(const float kp, const float ki, const float kd, const float isat)
  {
    setGain(kp, ki, kd);
    this->isat = isat;
  }

  void setLimit(const float min, const float max)
  {
    this->min = min;
    this->max = max;
  }

  float getIntegrator() const
  {
    return ei;
  }

protected:
  float kp;
  float ki;
  float kd;
  float isat;
  float ei;
  float ep;
  float u;
  float min, max;
};

#endif  // CRANE_BEHAVIOR_TREE__UTILS__PID_CONTROLLER_HPP_
