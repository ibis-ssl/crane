// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include "crane_physics/bang_bang_trajectory.hpp"

TEST(BangBangTrajectoryTest, Test1D)
{
  crane::BangBangTrajectory1D traj;
  // Move from 0 to 10 with v0=0, max_vel=2, max_acc=1
  traj.generate(0.0, 10.0, 0.0, 2.0, 1.0);

  // Acceleration phase: t=0 to 2. v=t. x=0.5*t^2.
  // At t=2: v=2, x=2.
  // Constant vel phase: t=2 to 5. v=2. x=2 + 2*(t-2).
  // At t=5: v=2, x=2+6=8.
  // Deceleration phase: t=5 to 7. v=2 - (t-5). x=8 + 2*(t-5) - 0.5*(t-5)^2.
  // At t=7: v=0, x=8 + 4 - 2 = 10.

  EXPECT_NEAR(traj.getTotalTime(), 7.0, 1e-3);
  EXPECT_NEAR(traj.getPosition(0.0), 0.0, 1e-3);
  EXPECT_NEAR(traj.getPosition(2.0), 2.0, 1e-3);
  EXPECT_NEAR(traj.getPosition(5.0), 8.0, 1e-3);
  EXPECT_NEAR(traj.getPosition(7.0), 10.0, 1e-3);
  EXPECT_NEAR(traj.getVelocity(2.0), 2.0, 1e-3);  // End of accel
  EXPECT_NEAR(traj.getVelocity(3.5), 2.0, 1e-3);  // Middle
}

TEST(BangBangTrajectoryTest, Test2D)
{
  crane::BangBangTrajectory2D traj;
  // Move diagonal (10, 10)
  // X and Y should be identical if parameters are identical
  // Due to vector partitioning (vmax/sqrt(2)), the time will be longer than 7.0s
  // Roughly 9.07s
  traj.generate(Eigen::Vector2d(0, 0), Eigen::Vector2d(10, 10), Eigen::Vector2d(0, 0), 2.0, 1.0);

  EXPECT_GT(traj.getTotalTime(), 7.0);
  EXPECT_NEAR(traj.x.getTotalTime(), traj.y.getTotalTime(), 0.1);  // Synced

  double t_mid = traj.getTotalTime() / 2.0;
  EXPECT_NEAR(traj.getPosition(t_mid).x(), traj.getPosition(t_mid).y(), 1e-3);

  EXPECT_NEAR(traj.getPosition(traj.getTotalTime()).x(), 10.0, 1e-3);
  EXPECT_NEAR(traj.getPosition(traj.getTotalTime()).y(), 10.0, 1e-3);
}

TEST(BangBangTrajectoryTest, Test2D_Asymmetric)
{
  crane::BangBangTrajectory2D traj;
  // Move (10, 5). X is longer.
  // Y should slow down to match X time.

  traj.generate(Eigen::Vector2d(0, 0), Eigen::Vector2d(10, 5), Eigen::Vector2d(0, 0), 2.0, 1.0);

  EXPECT_NEAR(traj.x.getTotalTime(), traj.y.getTotalTime(), 0.1);

  double totalTime = traj.getTotalTime();
  EXPECT_NEAR(traj.getPosition(totalTime).x(), 10.0, 1e-3);
  EXPECT_NEAR(traj.getPosition(totalTime).y(), 5.0, 1e-3);
}
