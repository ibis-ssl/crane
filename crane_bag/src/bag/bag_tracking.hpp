// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <vector>

#include "bag_reader.hpp"

namespace crane::bag
{

struct RobotState
{
  double t;
  int robot_id;
  double x, y, theta;
  double vx, vy, speed;
  bool detected;
  double dist_to_ball;
};

struct BallState
{
  double t;
  double x, y;
  double vx, vy, speed;
};

std::vector<RobotState> track_robot(
  const BagData & data, int robot_id, bool is_ours = true, double interval = 0.1);

std::vector<BallState> track_ball(const BagData & data, double interval = 0.1);

}  // namespace crane::bag
