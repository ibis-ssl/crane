// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include "crane_local_planner/mppi.hpp"

TEST(MPPI, test) { ASSERT_NEAR(1, 1, 1e-5); }

TEST(MPPI, a)
{
  std::vector<Point> path;
  for (int i = 0; i < 5; i++) {
    path.push_back(Point(i, 0));
  }
  Pose2D goal;
  goal.pos = Point(5, 0);
  goal.theta = 0.;

  Pose2D pose;
  pose.pos = Point(0, 0);
  pose.theta = 0.;

  Pose2D vel;
  vel.pos = Point(1, 0);
  vel.theta = 0.;

  crane::Optimizer optimizer;
  optimizer.calcCmd(path, goal, pose, vel);
  ASSERT_NEAR(1, 1, 1e-5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
