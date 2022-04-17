// Copyright (c) 2022 ibis-ssl
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

#ifndef CRANE_BT_EXECUTOR__UTILS__VELOCITY_PLANNER_HPP_
#define CRANE_BT_EXECUTOR__UTILS__VELOCITY_PLANNER_HPP_

#include "crane_geometry/boost_geometry.hpp"

class VelocityPlanner
{
public:
  void initilize(float freq, float max_vel, float max_acc)
  {
    dt_sec_ = 1.f / freq;
    max_velocity_ = max_vel;
    max_accelaration_ = max_acc;
    current_velocity_ = 0.f;
  }
  void update(Point current_pos, Point target_pos, float target_vel = 0.f)
  {
    float dist = (current_pos - target_pos).norm();
    update(dist, target_vel);
  }
  void update(float distance, float target_vel = 0.f)
  {
    float velocity_limit = std::sqrt(2 * max_accelaration_ * distance) + target_vel;
    if (current_velocity_ > velocity_limit)
    {
      // stable
      current_velocity_ = velocity_limit;
    }
    else
    {
      // accelaration
      current_velocity_ += max_accelaration_ * dt_sec_;
    }

    // deccelaration
    if (current_velocity_ > max_velocity_)
    {
      current_velocity_ = max_velocity_;
    }
  }

  float getVelocity()
  {
    return current_velocity_;
  }

private:
  float dt_sec_;
  float max_velocity_;
  float max_accelaration_;
  float current_velocity_;
};
#endif  // CRANE_BT_EXECUTOR__UTILS__VELOCITY_PLANNER_HPP_
