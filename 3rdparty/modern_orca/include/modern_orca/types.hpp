// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <array>
#include <cmath>
#include <concepts>
#include <limits>
#include <type_traits>

// crane_geometryのVector2をメインのベクトル型として使用
#include <crane_geometry/boost_geometry.hpp>

namespace crane::modern_orca
{
using AgentId = std::size_t;

constexpr double EPSILON = std::numeric_limits<double>::epsilon() * 100;
constexpr double PI = 3.14159265358979323846;
constexpr double INF = std::numeric_limits<double>::infinity();

// crane_geometry::Vector2互換性のためのヘルパー関数
inline auto distance(const Vector2 & a, const Vector2 & b) noexcept -> double
{
  return (a - b).norm();
}

inline auto distanceSquared(const Vector2 & a, const Vector2 & b) noexcept -> double
{
  return (a - b).squaredNorm();
}

inline auto dot(const Vector2 & a, const Vector2 & b) noexcept -> double { return a.dot(b); }

inline auto cross(const Vector2 & a, const Vector2 & b) noexcept -> double
{
  return a.x() * b.y() - a.y() * b.x();
}

inline auto lerp(const Vector2 & a, const Vector2 & b, double t) noexcept -> Vector2
{
  return a + t * (b - a);
}

// Vector2互換性のための追加ヘルパー関数
inline auto isZero(const Vector2 & v, double tolerance = EPSILON) noexcept -> bool
{
  return v.norm() < tolerance;
}

inline auto perpendicular(const Vector2 & v) noexcept -> Vector2 { return Vector2(-v.y(), v.x()); }

struct HalfPlane
{
  Vector2 normal;
  Vector2 point;

  HalfPlane() = default;
  constexpr HalfPlane(const Vector2 & n, const Vector2 & p) noexcept
  : normal(n.normalized()), point(p)
  {
  }

  constexpr auto signedDistance(const Vector2 & test_point) const noexcept -> double
  {
    return dot(normal, test_point - point);
  }

  constexpr bool contains(const Vector2 & test_point, double tolerance = EPSILON) const noexcept
  {
    return signedDistance(test_point) >= -tolerance;
  }

  constexpr auto project(const Vector2 & test_point) const noexcept -> Vector2
  {
    const auto dist = signedDistance(test_point);
    return test_point - dist * normal;
  }
};

struct Line
{
  Vector2 point;
  Vector2 direction;

  Line() = default;
  constexpr Line(const Vector2 & p, const Vector2 & d) noexcept
  : point(p), direction(d.normalized())
  {
  }

  static constexpr auto fromTwoPoints(const Vector2 & p1, const Vector2 & p2) noexcept -> Line
  {
    return {p1, p2 - p1};
  }

  constexpr auto at(double t) const noexcept -> Vector2 { return point + t * direction; }

  constexpr auto closestPoint(const Vector2 & test_point) const noexcept -> Vector2
  {
    const auto t = dot(test_point - point, direction);
    return at(t);
  }

  constexpr auto distanceTo(const Vector2 & test_point) const noexcept -> double
  {
    return distance(test_point, closestPoint(test_point));
  }
};
}  // namespace crane::modern_orca
