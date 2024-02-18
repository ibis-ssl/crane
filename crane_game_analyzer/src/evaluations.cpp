// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_game_analyzer/evaluations/evaluations.hpp>

namespace crane::evaluation
{
double getNextTargetVisibleScore(
  Point p, Point next_target, WorldModelWrapper::SharedPtr world_model)
{
  auto ball_line_norm = (next_target - p).normalized();
  // 次のパスライン単位ベクトルと敵方向の内積で評価（パスラインと敵方向のパスコースから角度差分のcos）
  double max_cos = 0.0;
  for (auto enemy : world_model->theirs.robots) {
    if (enemy->available) {
      auto norm = (enemy->pose.pos - p).normalized();
      double cos = ball_line_norm.dot(norm);
      max_cos = std::max(max_cos, cos);
    }
  }
  // 角度が大きい(cosが小さい)ほど安全
  return 1 - max_cos;
}

double getReachScore(
  RobotIdentifier id, Point p, double nearest_dist, WorldModelWrapper::SharedPtr world_model)
{
  auto & pos = world_model->getRobot(id)->pose.pos;
  double distance = (p - pos).norm();
  return nearest_dist / distance;
}

double getAngleScore(
  RobotIdentifier id, Point p, Point next_target, WorldModelWrapper::SharedPtr world_model)
{
  // 入射角＋反射角のcosを計算(内積を使用)
  auto & pos = world_model->getRobot(id)->pose.pos;
  auto current_pass_line = (world_model->ball.pos - p).normalized();
  auto next_pass_line = (next_target - p).normalized();
  float dot = current_pass_line.dot(next_pass_line);
  return dot;
}

double getEnemyDistanceScore(Point p, WorldModelWrapper::SharedPtr world_model, double max_dist)
{
  // 一番近い敵ロボットからの距離を求める
  double min_sq_dist = 100.0f;
  for (auto enemy : world_model->theirs.robots) {
    if (enemy->available) {
      double sq_dist = (enemy->pose.pos - p).squaredNorm();
      min_sq_dist = std::min(min_sq_dist, sq_dist);
    }
  }
  //最大距離設定(それ以上は評価値を1(安全)とする)
  min_sq_dist = std::min(min_sq_dist, 3.0 * 3.0);
  return sqrt(min_sq_dist) / 3.0;
}
}  // namespace crane::evaluation
