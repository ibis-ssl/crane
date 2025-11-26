// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_game_analyzer/evaluations/evaluations.hpp>
#include <range/v3/algorithm/max.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/transform.hpp>

namespace crane::evaluation
{
auto getNextTargetVisibleScore(Point p, Point next_target, WorldModelWrapper::SharedPtr world_model)
  -> double
{
  auto ball_line_norm = (next_target - p).normalized();
  // 次のパスライン単位ベクトルと敵方向の内積で評価（パスラインと敵方向のパスコースから角度差分のcos）
  auto cos_values = world_model->theirs().robots |
                    ranges::views::filter([](const auto & enemy) { return enemy->available(); }) |
                    ranges::views::transform([&](const auto & enemy) {
                      auto norm = (enemy->pose.pos - p).normalized();
                      return ball_line_norm.dot(norm);
                    }) |
                    ranges::to<std::vector>();
  auto max_cos = cos_values.empty() ? 0.0 : ranges::max(cos_values);
  // 角度が大きい(cosが小さい)ほど安全
  return 1 - max_cos;
}

auto getReachScore(
  RobotIdentifier id, Point p, double nearest_dist, WorldModelWrapper::SharedPtr world_model)
  -> double
{
  auto & pos = world_model->getRobot(id)->pose.pos;
  double distance = (p - pos).norm();
  return nearest_dist / distance;
}

auto getAngleScore(
  RobotIdentifier, Point p, Point next_target, WorldModelWrapper::SharedPtr world_model) -> double
{
  // 入射角＋反射角のcosを計算(内積を使用)
  auto current_pass_line = (world_model->ball().pos - p).normalized();
  auto next_pass_line = (next_target - p).normalized();
  float dot = current_pass_line.dot(next_pass_line);
  return dot;
}

auto getEnemyDistanceScore(Point p, WorldModelWrapper::SharedPtr world_model, double) -> double
{
  // 一番近い敵ロボットからの距離を求める
  auto sq_distances = world_model->theirs().robots |
                      ranges::views::filter([](const auto & enemy) { return enemy->available(); }) |
                      ranges::views::transform(
                        [&](const auto & enemy) { return (enemy->pose.pos - p).squaredNorm(); }) |
                      ranges::to<std::vector>();
  auto min_sq_dist = sq_distances.empty() ? 100.0 : ranges::min(sq_distances);
  // 最大距離設定(それ以上は評価値を1(安全)とする)
  min_sq_dist = std::min(min_sq_dist, 3.0 * 3.0);
  return std::sqrt(min_sq_dist) / 3.0;
}
}  // namespace crane::evaluation
