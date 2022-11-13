// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
