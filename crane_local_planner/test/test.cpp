// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include "crane_local_planner/mppi.hpp"

TEST(MPPI, test) { ASSERT_NEAR(1, 1, 1e-5); }
TEST(MPPI, aaa)
{
  constexpr int BATCH = 5;
  constexpr int STEP = 3;
  Eigen::Matrix<int, BATCH, STEP> mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
  std::cout << "mat:" << std::endl;
  std::cout << " row(行): " << mat.rows() << std::endl;
  std::cout << " col(列): " << mat.cols() << std::endl;
  std::cout << mat << std::endl;

  //  //  colwise()は列ごとに処理する
  //  std::cout << "mat.colwise().sum()" << std::endl;
  //  auto colwise = mat.colwise().sum();
  //  std::cout << "size: " << colwise.rows() << "x" << colwise.cols() << std::endl;
  //  std::cout << colwise << std::endl;
  //
  //  //  rowwise()は行ごとに処理する
  //  std::cout << "mat.rowwise().sum()" << std::endl;
  //  auto rowwise = mat.rowwise().sum();
  //  std::cout << "size: " << rowwise.rows() << "x" << rowwise.cols() << std::endl;
  //  std::cout << rowwise << std::endl;

  auto mean = mat.rowwise().mean();
  std::cout << "mat.rowwise().mean()" << std::endl;
  std::cout << "size: " << mean.rows() << "x" << mean.cols() << std::endl;
  std::cout << mean << std::endl;

  auto pow = mean.array().pow(2);
  std::cout << "mean.array().pow(2)" << std::endl;
  std::cout << "size: " << pow.rows() << "x" << pow.cols() << std::endl;
  std::cout << pow << std::endl;

  ASSERT_NEAR(1, 1, 1e-5);
}

TEST(MPPI, simple)
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

  crane::Optimizer<100, 56> optimizer;
  optimizer.calcCmd(path, goal, pose, vel);
  ASSERT_NEAR(1, 1, 1e-5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
