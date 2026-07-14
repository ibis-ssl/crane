// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_
#define CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_

#include <algorithm>
#include <cmath>  // For std::fabs and std::sqrt
#include <crane_geometry/boost_geometry.hpp>
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

inline auto getAngleDiff(const Pose2D & pose1, const Pose2D & pose2) -> double
{
  return getAngleDiff(pose1.theta, pose2.theta);
}

inline auto getAngleDiff(const Pose2D & pose1, const double angle_rad) -> double
{
  return getAngleDiff(pose1.theta, angle_rad);
}

inline auto getAngleDiff(const double angle_rad, const Pose2D & pose1) -> double
{
  return getAngleDiff(angle_rad, pose1.theta);
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
  vertical_v.x() = v.y();   // Corrected syntax
  vertical_v.y() = -v.x();  // Corrected syntax
  return vertical_v;
}

inline auto getReachTime(
  const double distance, const double v0, const double acc, const double max_vel) -> double
{
  // x = v0*t + 1/2*a*t^2 より
  double t = (sqrt(v0 * v0 + 2.0f * acc * distance) - v0) / acc;
  if (max_vel == -1.f) {
    return t;
  } else {
    double acc_end_time = (max_vel - v0) / acc;
    if (t > acc_end_time) {
      return (distance + 0.5f * std::pow(max_vel - v0, 2.f) / acc) / max_vel;
    } else {
      return t;
    }
  }
}

inline auto getIntersections(const Segment & segment1, const Segment & segment2)
  -> std::vector<Point>
{
  std::vector<Point> intersections;
  bg::intersection(segment1, segment2, intersections);
  return intersections;
}

inline auto getIntersections(const Circle & circle, const Segment & segment) -> std::vector<Point>
{
  std::vector<Point> intersections;
  double distance = bg::distance(circle, segment);
  if (distance > circle.radius) {
    // 交差しない
    return intersections;
  } else {
    // 交差する
    // 交点を求める
    Vector2 norm_vec = getVerticalVec(segment.second - segment.first).normalized();
    if (
      ((circle.center + norm_vec) - segment.first).norm() >
      ((circle.center - norm_vec) - segment.first).norm()) {
      norm_vec = -norm_vec;
    }
    double d = std::sqrt(circle.radius * circle.radius - distance * distance);
    Vector2 seg_norm = (segment.second - segment.first).normalized();
    Point p1 = circle.center + norm_vec * distance + seg_norm * d;
    Point p2 = circle.center + norm_vec * distance - seg_norm * d;

    // 交点が線分上にあるか確認
    if (
      (p1 - segment.first).dot(segment.second - segment.first) > 0 &&
      (p1 - segment.second).dot(segment.first - segment.second) > 0) {
      intersections.push_back(p1);
    }

    if (
      (p2 - segment.first).dot(segment.second - segment.first) > 0 &&
      (p2 - segment.second).dot(segment.first - segment.second) > 0) {
      intersections.push_back(p2);
    }

    return intersections;
  }
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

inline auto getCircle(const Point & p1, const Point & p2, const Point & p3) -> std::optional<Circle>
{
  // Using the formula from https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_2
  double D =
    2 * (p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));

  if (std::fabs(D) < 1e-9) {  // Points are collinear
    return std::nullopt;
  }

  Circle circle;
  double p1_sq = p1.x() * p1.x() + p1.y() * p1.y();
  double p2_sq = p2.x() * p2.x() + p2.y() * p2.y();
  double p3_sq = p3.x() * p3.x() + p3.y() * p3.y();

  circle.center.x() =
    (p1_sq * (p2.y() - p3.y()) + p2_sq * (p3.y() - p1.y()) + p3_sq * (p1.y() - p2.y())) /
    D;  // Corrected syntax
  circle.center.y() =
    (p1_sq * (p3.x() - p2.x()) + p2_sq * (p1.x() - p3.x()) + p3_sq * (p2.x() - p1.x())) /
    D;  // Corrected syntax

  circle.radius = std::sqrt(
    (circle.center.x() - p1.x()) * (circle.center.x() - p1.x()) +
    (circle.center.y() - p1.y()) * (circle.center.y() - p1.y()));
  return circle;
}

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
  // 進捗（回り込みの達成度）を評価
  Vector2 a = (desired_opposite - ball).normalized();
  Vector2 b = (from - ball).normalized();
  double dot = a.dot(b);
  double progress = std::clamp((1.0 - dot) / 2.0, 0.0, 1.0);  // [0,1]

  // 有効オフセット（初期はmax_offset、完了でbase_offset）
  double offset_eff = max_offset + (base_offset - max_offset) * progress;
  offset_eff =
    std::clamp(offset_eff, std::min(base_offset, max_offset), std::max(base_offset, max_offset));

  return computeAroundBallApproachTarget(ball, desired_opposite, from, offset_eff, epsilon);
}
}  // namespace crane

#endif  // CRANE_GEOMETRY__GEOMETRY_OPERATIONS_HPP_
