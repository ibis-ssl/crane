// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

export module crane_basics:ball_model;

import :boost_geometry;  // Point type from here

// <optional>, <range/v3/all.hpp>, <utility>, <vector> removed.
// Assumed from crane_basics.cppm global fragment.

export namespace crane
{
export inline auto getFutureBallPosition(
  Point ball_pos, Point ball_vel, double t, double deceleration = 0.5) -> Point
{
  // 指定時間までに停止する場合
  if (ball_vel.norm() - deceleration * t < 0.) {
    double stop_time = ball_vel.norm() / deceleration;
    return ball_pos + ball_vel * stop_time -
           0.5 * stop_time * stop_time * deceleration * ball_vel.normalized();
  } else {
    return ball_pos + ball_vel * t - 0.5 * t * t * deceleration * ball_vel.normalized();
  }
}

/**
 * ボールが指定された距離に到達するのにかかる時間を計算します
 * @param distance_to_target 目標までの距離
 * @param current_ball_vel 現在のボール速度の大きさ
 * @param deceleration 減速度（デフォルトは0.5）
 * @return ボールが目標距離に到達するまでの時間
 */
export inline auto getBallReachTime(
  double distance_to_target, double current_ball_vel, double deceleration = 0.5)
  -> std::optional<double>
{
  // ボールが完全に停止するまでの時間
  double stop_time = current_ball_vel / deceleration;

  // ボールが停止するまでに移動する距離
  double max_distance = current_ball_vel * stop_time - 0.5 * deceleration * stop_time * stop_time;

  // 目標距離が最大到達距離より大きい場合、到達不可能
  if (distance_to_target > max_distance) {
    return std::nullopt;
  }

  // ボールが目標距離に到達する時間を計算する
  // 二次方程式: -0.5 * deceleration * t^2 + current_ball_vel * t - distance_to_target = 0 を解く
  // at^2 + bt + c = 0 の形式で：
  // a = -0.5 * deceleration
  // b = current_ball_vel
  // c = -distance_to_target

  double a = -0.5 * deceleration;
  double b = current_ball_vel;
  double c = -distance_to_target;

  // 判別式
  double discriminant = b * b - 4 * a * c;

  // 解がない場合（通常はここには到達しない）
  if (discriminant < 0) {
    return std::nullopt;
  }

  // 二次方程式の解：(-b ± sqrt(discriminant)) / (2a)
  // ここでは、小さい方の解（より早く到達する時間）を選ぶ
  double t1 = (-b + sqrt(discriminant)) / (2 * a);
  double t2 = (-b - sqrt(discriminant)) / (2 * a);

  // 物理的に意味のある解（正の時間）を選ぶ
  if (t1 > 0 && t1 <= stop_time) {
    return t1;
  } else if (t2 > 0 && t2 <= stop_time) {
    return t2;
  }

  // 通常はここに到達しないはずだが、安全のため
  return std::nullopt;
}

export inline auto generateSequence(double start, double end, double step) -> std::vector<double>
{
  int size = (end - start) / step + 1;
  return ranges::views::iota(0, size) |
         ranges::views::transform([&](int i) -> double { return start + i * step; }) |
         ranges::to<std::vector>();
}

export inline auto getBallSequence(double t_horizon, double t_step, Point ball_pos, Point ball_vel)
  -> std::vector<std::pair<Point, double>>
{
  auto t_ball_sequence = generateSequence(0.0, t_horizon, t_step);
  return t_ball_sequence | ranges::views::transform([&](double t) {
           return std::make_pair(getFutureBallPosition(ball_pos, ball_vel, t), t);
         }) |
         ranges::views::transform(
           [](const auto & p) { return std::make_pair(p.first, p.second); }) |
         ranges::to<std::vector>();
}
}  // namespace crane
