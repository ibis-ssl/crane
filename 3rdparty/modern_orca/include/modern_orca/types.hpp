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

// Use crane_basics Vector2d as the primary vector type
#include <crane_basics/vector2d.hpp>

namespace crane::modern_orca
{
using AgentId = std::size_t;

constexpr double EPSILON = std::numeric_limits<double>::epsilon() * 100;
constexpr double PI = 3.14159265358979323846;
constexpr double INF = std::numeric_limits<double>::infinity();

// Helper functions for crane_basics::Vector2d compatibility
inline auto distance(const Vector2d & a, const Vector2d & b) noexcept -> double
{
  return (a - b).norm();
}

inline auto distanceSquared(const Vector2d & a, const Vector2d & b) noexcept -> double
{
  return (a - b).squaredNorm();
}

inline auto dot(const Vector2d & a, const Vector2d & b) noexcept -> double { return a.dot(b); }

inline auto cross(const Vector2d & a, const Vector2d & b) noexcept -> double
{
  return a.x() * b.y() - a.y() * b.x();
}

inline auto lerp(const Vector2d & a, const Vector2d & b, double t) noexcept -> Vector2d
{
  return a + t * (b - a);
}

// Additional helper functions for Vector2d compatibility
inline auto isZero(const Vector2d & v, double tolerance = EPSILON) noexcept -> bool
{
  return v.norm() < tolerance;
}

inline auto perpendicular(const Vector2d & v) noexcept -> Vector2d
{
  return Vector2d(-v.y(), v.x());
}

struct HalfPlane
{
  Vector2d normal;
  Vector2d point;

  HalfPlane() = default;
  constexpr HalfPlane(const Vector2d & n, const Vector2d & p) noexcept
  : normal(n.normalized()), point(p)
  {
  }

  constexpr auto signedDistance(const Vector2d & test_point) const noexcept -> double
  {
    return dot(normal, test_point - point);
  }

  constexpr bool contains(const Vector2d & test_point, double tolerance = EPSILON) const noexcept
  {
    return signedDistance(test_point) >= -tolerance;
  }

  constexpr auto project(const Vector2d & test_point) const noexcept -> Vector2d
  {
    const auto dist = signedDistance(test_point);
    return test_point - dist * normal;
  }
};

struct Line
{
  Vector2d point;
  Vector2d direction;

  Line() = default;
  constexpr Line(const Vector2d & p, const Vector2d & d) noexcept
  : point(p), direction(d.normalized())
  {
  }

  static constexpr auto fromTwoPoints(const Vector2d & p1, const Vector2d & p2) noexcept -> Line
  {
    return {p1, p2 - p1};
  }

  constexpr auto at(double t) const noexcept -> Vector2d { return point + t * direction; }

  constexpr auto closestPoint(const Vector2d & test_point) const noexcept -> Vector2d
  {
    const auto t = dot(test_point - point, direction);
    return at(t);
  }

  constexpr auto distanceTo(const Vector2d & test_point) const noexcept -> double
  {
    return distance(test_point, closestPoint(test_point));
  }
};
}  // namespace crane::modern_orca
