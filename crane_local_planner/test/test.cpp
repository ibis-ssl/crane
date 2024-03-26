// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>
#include <matplotlibcpp17/pyplot.h>

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

  //  colwise()は列ごとに処理する
  std::cout << "mat.colwise().sum()" << std::endl;
  auto colwise = mat.colwise().sum();
  std::cout << "size: " << colwise.rows() << "x" << colwise.cols() << std::endl;
  std::cout << colwise << std::endl;

  //  rowwise()は行ごとに処理する
  std::cout << "mat.rowwise().sum()" << std::endl;
  auto rowwise = mat.rowwise().sum();
  std::cout << "size: " << rowwise.rows() << "x" << rowwise.cols() << std::endl;
  std::cout << rowwise << std::endl;

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

  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(10, 10), 0.1, grid_map::Position(0, 0));
  map.add("cost", 0.0);
  // Position(3.0, 3.0)
  //  map.atPosition("cost", grid_map::Position(3.0, 3.0)) = 2.0;

  crane::Optimizer<100, 40> optimizer;

  crane::models::Path path_;
  {
    path_.reset(path.size());
    for (int i = 0; i < path.size(); i++) {
      path_.x(i) = path[i].x();
      path_.y(i) = path[i].y();
      path_.yaws(i) = goal.theta;
    }
  }

  pybind11::scoped_interpreter guard{};
  //  using matplotlibcpp17::Args;
  auto plt = matplotlibcpp17::pyplot::import();

  //  namespace plt = matplotlibcpp;
  //  plt.title("Fig. 1 : A nice figure");

  // optimizer.optimize(path_, goal);
  crane::models::Trajectories<100, 40> trajectories_;
  for (int i = 0; i < 30; i++) {
    auto [state, trajectories] = optimizer.generateNoisedTrajectories();
    trajectories_ = trajectories;
    auto costs = optimizer.getScore(state, trajectories, path_, goal, map, "cost");
    std::cout << costs << std::endl;
    optimizer.updateControlSequence(state, costs);

    std::cout << costs << std::endl;

    plt.clf();
    for (int i = 0; i < trajectories_.x.rows(); i++) {
      Eigen::RowVectorXf raw_x = trajectories_.x.row(i);
      Eigen::RowVectorXf raw_y = trajectories_.y.row(i);
      std::vector<float> x(raw_x.data(), raw_x.data() + raw_x.size());
      std::vector<float> y(raw_y.data(), raw_y.data() + raw_y.size());
      plt.plot(Args(x, y), Kwargs("alpha"_a = std::clamp(static_cast<double>(costs[i]), 0., 1.)));
    }
    //    plt::xlabel("x [m]");
    //    plt::ylabel("y [m]");
    //    plt::xlim(-1, 5);
    //    plt::set_aspect_equal();
    //    plt::pause(0.01);
  }
  plt.show();

  ASSERT_NEAR(1, 1, 1e-5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
