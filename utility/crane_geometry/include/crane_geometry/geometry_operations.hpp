// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_
#define CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_

#include <algorithm>
#include <cmath>
#include <crane_geometry/boost_geometry.hpp>
#include <limits>
#include <optional>
#include <vector>

namespace crane
{
inline auto isInBox(const Box & box, const Point & p) -> bool { return bg::within(p, box); }

inline auto isInBox(Box box, const Point & p, const double offset) -> bool
{
  box.max_corner() += Point(offset, offset);
  box.min_corner() -= Point(offset, offset);
  return bg::within(p, box);
}

inline auto createBox(const Point & p1, const Point & p2) -> Box
{
  Box box;
  box.min_corner() = Point(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()));
  box.max_corner() = Point(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()));
  return box;
}

/**
 * @brief 度からラジアンへの変換 (constexpr)
 *
 * @tparam T 数値型（デフォルト: double）
 * @param deg 角度（度）
 * @return T 角度（ラジアン）
 */
template <typename T = double>
constexpr auto deg2rad(T deg) noexcept -> T
{
  return deg * static_cast<T>(M_PI / 180.0);
}

/**
 * @brief ラジアンから度への変換 (constexpr)
 *
 * @tparam T 数値型（デフォルト: double）
 * @param rad 角度（ラジアン）
 * @return T 角度（度）
 */
template <typename T = double>
constexpr auto rad2deg(T rad) noexcept -> T
{
  return rad * static_cast<T>(180.0 / M_PI);
}

inline auto getAngle(const Vector2 & vec) -> double { return atan2(vec.y(), vec.x()); }

inline auto normalizeAngle(double angle_rad) -> double
{
  while (angle_rad > M_PI) {
    angle_rad -= 2.0f * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0f * M_PI;
  }
  return angle_rad;
}

inline auto getAngleDiff(double angle_rad1, double angle_rad2) -> double
{
  angle_rad1 = normalizeAngle(angle_rad1);
  angle_rad2 = normalizeAngle(angle_rad2);
  if (abs(angle_rad1 - angle_rad2) > M_PI) {
    if (angle_rad1 - angle_rad2 > 0) {
      return angle_rad1 - angle_rad2 - 2.0f * M_PI;
    } else {
      return angle_rad1 - angle_rad2 + 2.0f * M_PI;
    }
  } else {
    return angle_rad1 - angle_rad2;
  }
}

inline auto getIntermediateAngle(double angle_rad1, double angle_rad2) -> double
{
  angle_rad1 = normalizeAngle(angle_rad1);
  angle_rad2 = normalizeAngle(angle_rad2);
  // 差がpiを超えている場合では平均を取るだけではダメ
  if (abs(angle_rad1 - angle_rad2) > M_PI) {
    return normalizeAngle((angle_rad1 + angle_rad2 + 2.0f * M_PI) / 2.0f);
  } else {
    return (angle_rad1 + angle_rad2) / 2.0f;
  }
}

inline auto getNormVec(const double angle) -> Vector2 { return {cos(angle), sin(angle)}; }

inline auto getVerticalVec(const Point & v) -> Point
{
  Point vertical_v;
  vertical_v.x() = v.y();
  vertical_v.y() = -v.x();
  return vertical_v;
}

/**
 * @brief 円と線分の交点を求める
 *
 * 線分を P(t) = segment.first + t * (segment.second - segment.first), t in [0, 1] と
 * 媒介変数表示し、|P(t) - center|^2 = radius^2 の二次方程式を直接解く。
 *
 * 垂線足を経由する構成は「円中心から線分への垂直距離」を必要とするが、
 * bg::distance(circle, segment) は円表面からの距離（0クランプ）を、
 * closest_point(segment, center) は線分にクランプされた距離を返すため、
 * どちらも垂直距離ではなく交点座標が破綻する。二次方程式解にはこの落とし穴がない。
 *
 * @return 交点。接する場合は1点、交差しない場合や線分の長さが0の場合は空。
 */
inline auto getIntersections(const Circle & circle, const Segment & segment) -> std::vector<Point>
{
  std::vector<Point> intersections;
  const Vector2 dir = segment.second - segment.first;
  const double a = dir.squaredNorm();
  // 長さ0の線分は方向が定義できないため交点なしとして扱う
  if (a < 1e-18) {
    return intersections;
  }
  const Vector2 to_start = Point(segment.first) - circle.center;
  const double b = 2.0 * to_start.dot(dir);
  const double c = to_start.squaredNorm() - circle.radius * circle.radius;
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return intersections;
  }
  const double sqrt_d = std::sqrt(discriminant);
  // a > 0 なので (-b - sqrt_d) が常に小さい方の解。始点側から終点側の順で並ぶ
  for (const double t : {(-b - sqrt_d) / (2.0 * a), (-b + sqrt_d) / (2.0 * a)}) {
    // 線分の外（無限直線上の交点）は除外する
    if (t < 0.0 || t > 1.0) {
      continue;
    }
    Point point = Point(segment.first) + dir * t;
    // 接する場合は重解となり同じ点が2つ得られるので1点に畳む
    if (not intersections.empty() && (intersections.back() - point).norm() < 1e-9) {
      continue;
    }
    intersections.push_back(point);
  }
  return intersections;
}

template <typename Geometry1, typename Geometry2>
inline auto getIntersections(const Geometry1 & geometry1, const Geometry2 & geometry2)
  -> std::vector<Point>
{
  std::vector<Point> intersections;
  bg::intersection(geometry1, geometry2, intersections);
  return intersections;
}

template <typename Geometry1, typename Geometry2>
inline auto getClosestPointAndDistance(const Geometry1 & geometry1, const Geometry2 & geometry2)
  -> ClosestPoint
{
  ClosestPoint result;
  bg::closest_point(geometry1, geometry2, result);
  return result;
}

/**
 * @brief 線分を separated_num + 1 等分する、端点を含まない separated_num 個の内分点を返す
 *
 * 点は始点 segment1.first 側から終点 segment1.second 側へ順に並ぶ。
 * separated_num が 0 以下のときと、線分の長さが 0 のときは空配列を返す。
 */
inline auto getSeparatedPoints(const Segment & segment1, int separated_num) -> std::vector<Point>
{
  std::vector<Point> points;
  Vector2 segment_vec = segment1.second - segment1.first;
  if (segment_vec.isZero()) {
    return points;
  }
  for (int i = 1; i <= separated_num; ++i) {
    points.push_back(segment1.first + segment_vec * i / static_cast<double>(separated_num + 1));
  }
  return points;
}

/**
 * @brief ボール回り込みのためのアプローチ目標を計算するユーティリティ
 *
 * 与えられた始点 from から、ボールの反対側（desired_opposite 方向）に offset だけ離れた
 * 基準点へ向かう線分を取り、その線分上でボールに最も近い点の方向へ offset だけオフセットする。
 *
 * 直感的には、ロボット→目標の経路がボールを横切る場合に、ボールに接しない滑らかな回り込み点を返す。
 *
 * @param ball            ボール位置
 * @param desired_opposite ボールから見た目標（例: 配置点、パスターゲット等）
 * @param from            開始点（例: ロボット位置）
 * @param offset          ボールから離れる距離（m）
 * @param epsilon         最近傍判定の閾値
 * @return Point          アプローチ目標位置
 */
inline auto computeAroundBallApproachTarget(
  const Point & ball, const Point & desired_opposite, const Point & from, double offset,
  double epsilon = 1e-4) -> Point
{
  Point base_target = ball + (ball - desired_opposite).normalized() * offset;
  Segment from_to_base{from, base_target};
  auto result = getClosestPointAndDistance(ball, from_to_base);
  if (result.distance > epsilon) {
    return ball + (result.closest_point - ball).normalized() * offset;
  } else {
    return base_target;
  }
}

/**
 * @brief 回り込み初期は大きめ、完了に向けて目標オフセットへ滑らかに収束させるアプローチ点計算
 *
 * ロボットの相対配置（ボールから見たdesired_opposite方向との整列度）を0..1の進捗として評価し、
 * offset_eff = lerp(max_offset, base_offset, progress) を用いて周回半径を逐次調整する。
 * そのうえで computeAroundBallApproachTarget を適用して接触回避かつ大回りし過ぎない経路を返す。
 *
 * progress は以下で計算:
 *   a = normalize(desired_opposite - ball)
 *   b = normalize(from - ball)
 *   progress = clamp((1 - dot(a, b)) / 2, 0, 1)
 *     - ロボットが目標と逆側に回り込めているほど 1 に近づく
 *
 * @param ball            ボール位置
 * @param desired_opposite ボールから見た目標（例: 配置点、パスターゲット等）
 * @param from            開始点（例: ロボット位置）
 * @param base_offset     最終的に収束させたいオフセット（INTERVAL最終値）
 * @param max_offset      初期に用いる上限オフセット（大回りの上限）
 * @param epsilon         最近傍判定の閾値
 * @return Point          アプローチ目標位置
 */
inline auto computeAroundBallApproachTargetDynamic(
  const Point & ball, const Point & desired_opposite, const Point & from, double base_offset,
  double max_offset, double epsilon = 1e-4) -> Point
{
  Vector2 a = (desired_opposite - ball).normalized();
  Vector2 b = (from - ball).normalized();
  double dot = a.dot(b);
  double progress = std::clamp((1.0 - dot) / 2.0, 0.0, 1.0);

  double offset_eff = max_offset + (base_offset - max_offset) * progress;
  offset_eff =
    std::clamp(offset_eff, std::min(base_offset, max_offset), std::max(base_offset, max_offset));

  return computeAroundBallApproachTarget(ball, desired_opposite, from, offset_eff, epsilon);
}

/**
 * @brief ベクトルのノルムを指定した上限値以下に制限する（ノルム飽和）
 *
 * @param vec 対象2Dベクトル
 * @param max_norm 上限ノルム（0以下の場合は零ベクトルを返す）
 * @return Vector2 クランプされたベクトル
 */
inline auto clampNorm(const Vector2 & vec, double max_norm) -> Vector2
{
  if (max_norm <= 0.0) {
    return Vector2::Zero();
  }
  const double current_norm = vec.norm();
  if (current_norm > max_norm && current_norm > 1e-9) {
    return vec * (max_norm / current_norm);
  }
  return vec;
}

/**
 * @brief 2D点を min_p と max_p の範囲内にクランプする
 *
 * @param p 対象の2D点
 * @param min_p 最小境界点
 * @param max_p 最大境界点
 * @return Point クランプされた2D点
 */
inline auto clampPoint(const Point & p, const Point & min_p, const Point & max_p) -> Point
{
  return Point(std::clamp(p.x(), min_p.x(), max_p.x()), std::clamp(p.y(), min_p.y(), max_p.y()));
}

/**
 * @brief 2D点を Box 矩形領域内にクランプする
 *
 * @param p 対象の2D点
 * @param box クランプ対象の矩形領域
 * @return Point クランプされた2D点
 */
inline auto clampPoint(const Point & p, const Box & box) -> Point
{
  return clampPoint(p, box.min_corner(), box.max_corner());
}

/**
 * @brief 2D点を [min_x, max_x] x [min_y, max_y] の範囲内にクランプする
 *
 * @param p 対象の2D点
 * @param min_x X座標最小値
 * @param max_x X座標最大値
 * @param min_y Y座標最小値
 * @param max_y Y座標最大値
 * @return Point クランプされた2D点
 */
inline auto clampPoint(const Point & p, double min_x, double max_x, double min_y, double max_y)
  -> Point
{
  return Point(std::clamp(p.x(), min_x, max_x), std::clamp(p.y(), min_y, max_y));
}

/**
 * @brief 2D点を原点対称な境界 [-max_x, max_x] x [-max_y, max_y] 内にクランプする
 *
 * @param p 対象の2D点
 * @param max_x X座標の絶対値上限
 * @param max_y Y座標の絶対値上限
 * @return Point クランプされた2D点
 */
inline auto clampPoint(const Point & p, double max_x, double max_y) -> Point
{
  return clampPoint(p, -max_x, max_x, -max_y, max_y);
}

/**
 * @brief 中心からの距離を保ったまま、点を箱の内側へ滑らせる
 *
 * 点が箱の中ならそのまま返す。外なら、中心を通る同じ半径の円上で箱に入る点のうち、
 * 元の角度に最も近いものを返す（72 分割のサンプリング）。箱へ単純にクランプすると
 * 点が中心へ寄ってしまう場合（ボール周りの周回目標をフィールド内に収めるときなど）に使う。
 * 円が箱と交わらないときは箱へクランプした点を返す。
 *
 * @param center 円の中心
 * @param point 滑らせる点
 * @param box 収めたい箱
 * @return Point 箱の中に収めた点
 */
inline auto slideOntoCircleInsideBox(const Point & center, const Point & point, const Box & box)
  -> Point
{
  if (isInBox(box, point)) {
    return point;
  }
  const double radius = (point - center).norm();
  const double base_angle = getAngle(point - center);
  std::optional<Point> best;
  double best_angle_diff = std::numeric_limits<double>::infinity();
  constexpr int SAMPLES = 72;
  for (int i = 0; i < SAMPLES; ++i) {
    const double angle = base_angle + 2.0 * M_PI * i / SAMPLES;
    const Point candidate = center + radius * Vector2(std::cos(angle), std::sin(angle));
    if (!isInBox(box, candidate)) {
      continue;
    }
    const double angle_diff = std::abs(getAngleDiff(angle, base_angle));
    if (angle_diff < best_angle_diff) {
      best_angle_diff = angle_diff;
      best = candidate;
    }
  }
  if (best.has_value()) {
    return *best;
  }
  return clampPoint(point, box);
}
}  // namespace crane

#endif  // CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_
