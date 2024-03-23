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
  int time_steps = 10;  // 仮の時間ステップ数
  int batch_size = 5;   // 仮のバッチサイズ

  // 1 x time_stepsのベクトルをゼロで初期化
  Eigen::VectorXf control_sequence_vx = Eigen::VectorXf::Zero(time_steps);

  // batch_size x time_stepsの行列をゼロで初期化
  Eigen::MatrixXf noises_vx = Eigen::MatrixXf::Zero(batch_size, time_steps);

  // 新しい状態ベクトルを計算（ブロードキャスト足し算）
  // Eigenでは、行ベクトルを列ベクトルにブロードキャストする際にはreplicateを使用します。
  Eigen::MatrixXf state_cvx = noises_vx;
  for (int i = 0; i < state_cvx.rows(); i++) {
    state_cvx.row(i) += control_sequence_vx.transpose();
  }
  control_sequence_vx.transpose();

  // 結果の表示（オプション）
  std::cout << "state_cvx:\n" << state_cvx << std::endl;

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

  crane::Optimizer<1000, 56> optimizer;
  optimizer.calcCmd(path, goal, pose, vel);
  ASSERT_NEAR(1, 1, 1e-5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
