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

namespace modern_orca
{

template <typename T>
concept Numeric = std::integral<T> || std::floating_point<T>;

template <typename T>
concept FloatingPoint = std::floating_point<T>;

using Scalar = double;
using TimeStep = Scalar;
using AgentId = std::size_t;

constexpr Scalar EPSILON = std::numeric_limits<Scalar>::epsilon() * 100;
constexpr Scalar PI = 3.14159265358979323846;
constexpr Scalar INF = std::numeric_limits<Scalar>::infinity();

template <FloatingPoint T = Scalar>
class Vector2
{
public:
  using value_type = T;

  constexpr Vector2() noexcept : data_{T{0}, T{0}} {}
  constexpr Vector2(T x, T y) noexcept : data_{x, y} {}

  template <FloatingPoint U>
  constexpr explicit Vector2(const Vector2<U> & other) noexcept
  : data_{static_cast<T>(other.x()), static_cast<T>(other.y())}
  {
  }

  constexpr T x() const noexcept { return data_[0]; }
  constexpr T y() const noexcept { return data_[1]; }

  constexpr T & x() noexcept { return data_[0]; }
  constexpr T & y() noexcept { return data_[1]; }

  constexpr T operator[](std::size_t i) const noexcept { return data_[i]; }
  constexpr T & operator[](std::size_t i) noexcept { return data_[i]; }

  constexpr auto operator+() const noexcept -> Vector2 { return *this; }
  constexpr auto operator-() const noexcept -> Vector2 { return {-x(), -y()}; }

  constexpr auto operator+=(const Vector2 & rhs) noexcept -> Vector2 &
  {
    data_[0] += rhs.data_[0];
    data_[1] += rhs.data_[1];
    return *this;
  }

  constexpr auto operator-=(const Vector2 & rhs) noexcept -> Vector2 &
  {
    data_[0] -= rhs.data_[0];
    data_[1] -= rhs.data_[1];
    return *this;
  }

  constexpr auto operator*=(T scalar) noexcept -> Vector2 &
  {
    data_[0] *= scalar;
    data_[1] *= scalar;
    return *this;
  }

  constexpr auto operator/=(T scalar) noexcept -> Vector2 &
  {
    data_[0] /= scalar;
    data_[1] /= scalar;
    return *this;
  }

  constexpr auto squaredNorm() const noexcept -> T { return x() * x() + y() * y(); }

  auto norm() const noexcept -> T { return std::sqrt(squaredNorm()); }

  auto normalized() const noexcept -> Vector2
  {
    const auto n = norm();
    if (n < EPSILON) {
      return Vector2{T{0}, T{0}};
    }
    return *this / n;
  }

  constexpr auto dot(const Vector2 & other) const noexcept -> T
  {
    return x() * other.x() + y() * other.y();
  }

  constexpr auto cross(const Vector2 & other) const noexcept -> T
  {
    return x() * other.y() - y() * other.x();
  }

  auto angle() const noexcept -> T { return std::atan2(y(), x()); }

  auto rotate(T angle) const noexcept -> Vector2
  {
    const auto cos_a = std::cos(angle);
    const auto sin_a = std::sin(angle);
    return {x() * cos_a - y() * sin_a, x() * sin_a + y() * cos_a};
  }

  constexpr auto perpendicular() const noexcept -> Vector2 { return {-y(), x()}; }

  constexpr bool isZero(T tolerance = EPSILON) const noexcept
  {
    return squaredNorm() < tolerance * tolerance;
  }

  constexpr auto data() const noexcept -> const std::array<T, 2> & { return data_; }
  constexpr auto data() noexcept -> std::array<T, 2> & { return data_; }

private:
  std::array<T, 2> data_;
};

template <FloatingPoint T>
constexpr auto operator+(const Vector2<T> & lhs, const Vector2<T> & rhs) noexcept -> Vector2<T>
{
  return {lhs.x() + rhs.x(), lhs.y() + rhs.y()};
}

template <FloatingPoint T>
constexpr auto operator-(const Vector2<T> & lhs, const Vector2<T> & rhs) noexcept -> Vector2<T>
{
  return {lhs.x() - rhs.x(), lhs.y() - rhs.y()};
}

template <FloatingPoint T>
constexpr auto operator*(const Vector2<T> & vec, T scalar) noexcept -> Vector2<T>
{
  return {vec.x() * scalar, vec.y() * scalar};
}

template <FloatingPoint T>
constexpr auto operator*(T scalar, const Vector2<T> & vec) noexcept -> Vector2<T>
{
  return vec * scalar;
}

template <FloatingPoint T>
constexpr auto operator/(const Vector2<T> & vec, T scalar) noexcept -> Vector2<T>
{
  return {vec.x() / scalar, vec.y() / scalar};
}

template <FloatingPoint T>
constexpr bool operator==(const Vector2<T> & lhs, const Vector2<T> & rhs) noexcept
{
  return std::abs(lhs.x() - rhs.x()) < EPSILON && std::abs(lhs.y() - rhs.y()) < EPSILON;
}

template <FloatingPoint T>
constexpr bool operator!=(const Vector2<T> & lhs, const Vector2<T> & rhs) noexcept
{
  return !(lhs == rhs);
}

template <FloatingPoint T>
auto distance(const Vector2<T> & a, const Vector2<T> & b) noexcept -> T
{
  return (a - b).norm();
}

template <FloatingPoint T>
constexpr auto distanceSquared(const Vector2<T> & a, const Vector2<T> & b) noexcept -> T
{
  return (a - b).squaredNorm();
}

template <FloatingPoint T>
constexpr auto dot(const Vector2<T> & a, const Vector2<T> & b) noexcept -> T
{
  return a.dot(b);
}

template <FloatingPoint T>
constexpr auto cross(const Vector2<T> & a, const Vector2<T> & b) noexcept -> T
{
  return a.cross(b);
}

template <FloatingPoint T>
auto lerp(const Vector2<T> & a, const Vector2<T> & b, T t) noexcept -> Vector2<T>
{
  return a + t * (b - a);
}

using Vector2D = Vector2<Scalar>;

template <FloatingPoint T = Scalar>
struct HalfPlane
{
  Vector2<T> normal;
  Vector2<T> point;

  constexpr HalfPlane() = default;
  constexpr HalfPlane(const Vector2<T> & n, const Vector2<T> & p) noexcept
  : normal(n.normalized()), point(p)
  {
  }

  constexpr auto signedDistance(const Vector2<T> & test_point) const noexcept -> T
  {
    return dot(normal, test_point - point);
  }

  constexpr bool contains(const Vector2<T> & test_point, T tolerance = EPSILON) const noexcept
  {
    return signedDistance(test_point) >= -tolerance;
  }

  constexpr auto project(const Vector2<T> & test_point) const noexcept -> Vector2<T>
  {
    const auto dist = signedDistance(test_point);
    return test_point - dist * normal;
  }
};

using HalfPlaneD = HalfPlane<Scalar>;

template <FloatingPoint T = Scalar>
struct Line
{
  Vector2<T> point;
  Vector2<T> direction;

  constexpr Line() = default;
  constexpr Line(const Vector2<T> & p, const Vector2<T> & d) noexcept
  : point(p), direction(d.normalized())
  {
  }

  static constexpr auto fromTwoPoints(const Vector2<T> & p1, const Vector2<T> & p2) noexcept -> Line
  {
    return {p1, p2 - p1};
  }

  constexpr auto at(T t) const noexcept -> Vector2<T> { return point + t * direction; }

  constexpr auto closestPoint(const Vector2<T> & test_point) const noexcept -> Vector2<T>
  {
    const auto t = dot(test_point - point, direction);
    return at(t);
  }

  constexpr auto distanceTo(const Vector2<T> & test_point) const noexcept -> T
  {
    return distance(test_point, closestPoint(test_point));
  }
};

using LineD = Line<Scalar>;

}  // namespace modern_orca
