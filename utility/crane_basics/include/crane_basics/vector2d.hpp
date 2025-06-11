// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__VECTOR2D_HPP_
#define CRANE_BASICS__VECTOR2D_HPP_

#include <cmath>
#include <iostream>

namespace crane
{
class Vector2d
{
public:
  double x, y;

  // Constructors
  Vector2d() : x(0.0), y(0.0) {}
  Vector2d(double x_val, double y_val) : x(x_val), y(y_val) {}

  // Accessors
  double get_x() const { return x; }
  double get_y() const { return y; }
  void set_x(double val) { x = val; }
  void set_y(double val) { y = val; }

  // Vector operations
  Vector2d operator+(const Vector2d & other) const { return Vector2d(x + other.x, y + other.y); }
  Vector2d operator-(const Vector2d & other) const { return Vector2d(x - other.x, y - other.y); }
  Vector2d operator*(double scalar) const { return Vector2d(x * scalar, y * scalar); }
  double dot(const Vector2d & other) const { return x * other.x + y * other.y; }
  double norm() const { return std::sqrt(x * x + y * y); }
  Vector2d normalized() const
  {
    double n = norm();
    if (n > 0) {
      return Vector2d(x / n, y / n);
    }
    return Vector2d(0, 0);  // Or throw an exception for zero vector
  }

  // Overload for ostream to print Vector2d
  friend std::ostream & operator<<(std::ostream & os, const Vector2d & vec)
  {
    os << "(" << vec.x << ", " << vec.y << ")";
    return os;
  }
};
}  // namespace crane
#endif  // CRANE_BASICS__VECTOR2D_HPP_
