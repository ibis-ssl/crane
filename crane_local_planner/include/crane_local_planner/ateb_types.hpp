// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__ATEB_TYPES_HPP_
#define CRANE_LOCAL_PLANNER__ATEB_TYPES_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <limits>
#include <vector>

namespace crane::ateb
{

/// 統一障害物ラッパー（既存Circle/Box/Capsuleを包む）
struct Obstacle
{
  enum class Type { CIRCLE, BOX, CAPSULE };
  Type type = Type::CIRCLE;
  Circle circle{Point::Zero(), 0.0};
  Box box{};
  Capsule capsule{};
  bool skip_inflation = false;  ///< trueの場合、VG extract()で再膨張しない（既に膨張済みのBOX用）

  /// 解析的符号付き距離（負=内部）
  [[nodiscard]] double distance(const Point & p) const
  {
    switch (type) {
      case Type::CIRCLE:
        // bg::distance(p, circle) はCircleをpoint_tagとして扱い半径を無視するため、
        // 直接計算する（負値 = 内部）
        return (p - circle.center).norm() - circle.radius;
      case Type::BOX: {
        // Boxの場合: 外部は正の距離、内部は負で返す
        const double xmin = box.min_corner().x();
        const double xmax = box.max_corner().x();
        const double ymin = box.min_corner().y();
        const double ymax = box.max_corner().y();
        const double dx = std::max({xmin - p.x(), 0.0, p.x() - xmax});
        const double dy = std::max({ymin - p.y(), 0.0, p.y() - ymax});
        if (dx > 0.0 || dy > 0.0) {
          return std::sqrt(dx * dx + dy * dy);
        }
        // 内部: 最近境界面までの距離の負値
        const double d_left = p.x() - xmin;
        const double d_right = xmax - p.x();
        const double d_bottom = p.y() - ymin;
        const double d_top = ymax - p.y();
        return -std::min({d_left, d_right, d_bottom, d_top});
      }
      case Type::CAPSULE: {
        // bg::distance(p, capsule) はCapsuleをsegment_tagとして扱い半径を無視するため、
        // セグメントへの距離を直接計算して半径を引く（負値 = 内部）
        const auto [seg_dist, closest_pt] = getClosestPointAndDistance(capsule.segment, p);
        (void)closest_pt;
        return seg_dist - capsule.radius;
      }
      default:
        return 0.0;
    }
  }

  /// 解析的勾配（p から障害物表面に向かう単位ベクトルの逆 = 外向き法線）
  [[nodiscard]] Vector2 distanceGradient(const Point & p) const
  {
    switch (type) {
      case Type::CIRCLE: {
        const Vector2 diff = p - circle.center;
        const double norm = diff.norm();
        if (norm < 1e-9) return Vector2(1.0, 0.0);
        return diff / norm;
      }
      case Type::BOX: {
        const double xmin = box.min_corner().x();
        const double xmax = box.max_corner().x();
        const double ymin = box.min_corner().y();
        const double ymax = box.max_corner().y();
        const double dx = std::max({xmin - p.x(), 0.0, p.x() - xmax});
        const double dy = std::max({ymin - p.y(), 0.0, p.y() - ymax});
        if (dx > 0.0 || dy > 0.0) {
          // 外部: 最近傍点からの方向
          const double cx = std::clamp(p.x(), xmin, xmax);
          const double cy = std::clamp(p.y(), ymin, ymax);
          const Vector2 diff = p - Vector2(cx, cy);
          const double n = diff.norm();
          if (n < 1e-9) return Vector2(1.0, 0.0);
          return diff / n;
        }
        // 内部: 最近境界面の外向き法線
        const double d_left = p.x() - xmin;
        const double d_right = xmax - p.x();
        const double d_bottom = p.y() - ymin;
        const double d_top = ymax - p.y();
        const double min_d = std::min({d_left, d_right, d_bottom, d_top});
        if (min_d == d_left) return Vector2(-1.0, 0.0);
        if (min_d == d_right) return Vector2(1.0, 0.0);
        if (min_d == d_bottom) return Vector2(0.0, -1.0);
        return Vector2(0.0, 1.0);
      }
      case Type::CAPSULE: {
        // カプセルの最近傍点を求めてpからの方向を返す
        const auto [dist, closest] = getClosestPointAndDistance(capsule.segment, p);
        const Vector2 diff = p - closest;
        const double n = diff.norm();
        if (n < 1e-9) return Vector2(1.0, 0.0);
        return diff / n;
      }
      default:
        return Vector2(1.0, 0.0);
    }
  }

  /// 膨張したObstacleを返す（Minkowski sum）
  [[nodiscard]] Obstacle inflated(double margin) const
  {
    Obstacle result = *this;
    switch (type) {
      case Type::CIRCLE:
        result.circle.radius += margin;
        break;
      case Type::BOX: {
        result.box.min_corner().x() -= margin;
        result.box.min_corner().y() -= margin;
        result.box.max_corner().x() += margin;
        result.box.max_corner().y() += margin;
        break;
      }
      case Type::CAPSULE:
        result.capsule.radius += margin;
        break;
    }
    return result;
  }

  static Obstacle makeCircle(const Point & center, double radius)
  {
    Obstacle obs;
    obs.type = Type::CIRCLE;
    obs.circle = Circle{center, radius};
    return obs;
  }

  static Obstacle makeBox(const Box & box)
  {
    Obstacle obs;
    obs.type = Type::BOX;
    obs.box = box;
    return obs;
  }

  static Obstacle makeCapsule(const Capsule & capsule)
  {
    Obstacle obs;
    obs.type = Type::CAPSULE;
    obs.capsule = capsule;
    return obs;
  }
};

/// ホモトピークラス（A*で抽出した経路）
struct HomotopyClass
{
  std::vector<Point> waypoints;
  double cost = std::numeric_limits<double>::max();
};

/// 弾性バンドのノード
struct BandNode
{
  Point pos;
  bool fixed = false;  // start/goalはtrue
};

/// 弾性バンド
struct ElasticBand
{
  std::vector<BandNode> nodes;
  double total_cost = std::numeric_limits<double>::max();

  [[nodiscard]] bool isValid() const { return nodes.size() >= 2; }
};

/// 時間最適軌道の1点
struct TimePoint
{
  double t = 0.0;
  Point pos;
  Vector2 vel = Vector2::Zero();
  Vector2 accel = Vector2::Zero();
};

/// 時間最適軌道
struct TimeOptimalTrajectory
{
  std::vector<TimePoint> points;
  double total_time = 0.0;

  [[nodiscard]] bool isValid() const { return points.size() >= 2; }

  /// 時刻tにおける速度を線形補間で返す
  [[nodiscard]] Vector2 sampleVelocity(double t) const
  {
    if (points.empty()) return Vector2::Zero();
    if (t <= points.front().t) return points.front().vel;
    if (t >= points.back().t) return points.back().vel;

    for (size_t i = 1; i < points.size(); ++i) {
      if (t <= points[i].t) {
        const double dt = points[i].t - points[i - 1].t;
        if (dt < 1e-9) return points[i].vel;
        const double alpha = (t - points[i - 1].t) / dt;
        return points[i - 1].vel * (1.0 - alpha) + points[i].vel * alpha;
      }
    }
    return points.back().vel;
  }
};

/// ロボットごとの計画状態（フレーム間キャッシュ用）
struct RobotPlanState
{
  uint8_t robot_id = 0;
  ElasticBand best_band;
  TimeOptimalTrajectory trajectory;
  std::vector<HomotopyClass> cached_homotopies;
  Point cached_goal;
  bool warm_start_valid = false;
};

}  // namespace crane::ateb

#endif  // CRANE_LOCAL_PLANNER__ATEB_TYPES_HPP_
