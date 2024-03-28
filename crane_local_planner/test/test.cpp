// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>
#include <matplotlibcpp17/patches.h>
#include <matplotlibcpp17/pyplot.h>

#include <grid_map_core/iterators/CircleIterator.hpp>

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
    path.push_back(Point(-i / 5 * 0.4, 0));
  }
  Pose2D goal;
  goal.pos = Point(-0.4, 0);
  goal.theta = 0.;

  Pose2D pose;
  pose.pos = Point(0, 0);
  pose.theta = 0.;

  Pose2D vel;
  vel.pos = Point(-1, 0);
  vel.theta = 0.;

  grid_map::GridMap map;
  map.setGeometry(grid_map::Length(5, 5), 0.1, grid_map::Position(0, 0));
  map.add("cost", 0.0);

  constexpr double CIRCLE_X = 0.2;
  constexpr double CIRCLE_Y = 0.2;
  constexpr double CIRCLE_R = 0.15;
  for (grid_map::CircleIterator iterator(map, grid_map::Position(CIRCLE_X, CIRCLE_Y), CIRCLE_R);
       !iterator.isPastEnd(); ++iterator) {
    //    map.at("cost", *iterator) = 1.0;
  }

  constexpr int STEP = 40;
  constexpr int BATCH = 10;
  crane::Optimizer<BATCH, STEP> optimizer;
  optimizer.state.pose = pose;
  optimizer.state.velocity = vel;

  crane::models::Path path_;
  {
    path_.reset(path.size());
    for (int i = 0; i < path.size(); i++) {
      path_.x(i) = path[i].x();
      path_.y(i) = path[i].y();
      path_.yaws(i) = goal.theta;
    }
  }

  constexpr int PLOT = 0;
  constexpr int COST = 1;
  constexpr int ITERATION = 10;

  pybind11::scoped_interpreter guard{};
  //  using matplotlibcpp17::Args;
  auto plt = matplotlibcpp17::pyplot::import();
  auto [fig, axs] = plt.subplots(ITERATION, 2);

  // optimizer.optimize(path_, goal);
  crane::models::Trajectories<BATCH, STEP> trajectories_;
  for (int i = 0; i < ITERATION; i++) {
    std::cout << "iteration: " << i << std::endl;
    auto trajectories = optimizer.generateNoisedTrajectories();
    trajectories_ = trajectories;
    auto costs = optimizer.getScore(trajectories, path_, goal, map, "cost");
    std::cout << "raw cost:" << std::endl;
    std::cout << costs.transpose() << std::endl;
    optimizer.updateControlSequence(costs);

    std::cout << "cost:" << std::endl;
    std::cout << costs.transpose() << std::endl;

    //    plt.clf();
    axs[2 * i + PLOT].cla();
    axs[2 * i + COST].cla();
    auto c = matplotlibcpp17::patches::Circle(
      Args(py::make_tuple(CIRCLE_X, CIRCLE_Y), CIRCLE_R), Kwargs("fc"_a = "g", "ec"_a = "r"));
    axs[2 * i + PLOT].add_patch(Args(c.unwrap()));
    for (int j = 0; j < trajectories_.x.rows(); j++) {
      Eigen::RowVectorXf raw_x = trajectories_.x.row(j);
      Eigen::RowVectorXf raw_y = trajectories_.y.row(j);
      std::vector<float> x(raw_x.data(), raw_x.data() + raw_x.size());
      std::vector<float> y(raw_y.data(), raw_y.data() + raw_y.size());
      // ax.plot(Args(x, y), Kwargs("alpha"_a =
      // std::clamp(static_cast<double>(costs[i]), 0., 1.)));
      axs[2 * i + PLOT].plot(Args(x, y), Kwargs("label"_a = std::to_string(j)));
      axs[2 * i + PLOT].legend();
      axs[2 * i + PLOT].set_xlim(Args(-1, 2));
      axs[2 * i + PLOT].set_ylim(Args(-1, 1.5));
    }
    std::vector<int> index(BATCH);
    std::vector<float> cost_array(BATCH);
    for (int j = 0; j < BATCH; j++) {
      index[j] = j;
      cost_array[j] = costs(j);
    }
    axs[2 * i + COST].bar(Args(index, cost_array));
    plt.pause(Args(0.1));
  }
  plt.show();

  ASSERT_NEAR(1, 1, 1e-5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
