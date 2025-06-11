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
private:
  double x_, y_;  // Changed to private and named with underscore

public:
  // Constructors
  Vector2d() : x_(0.0), y_(0.0) {}
  Vector2d(double x_val, double y_val) : x_(x_val), y_(y_val) {}

  // Accessors (Eigen-like)
  double x() const { return x_; }
  double y() const { return y_; }
  void x(double val) { x_ = val; }  // Setter
  void y(double val) { y_ = val; }  // Setter

  // Vector operations
  Vector2d operator+(const Vector2d & other) const
  {
    return Vector2d(x_ + other.x_, y_ + other.y_);
  }
  Vector2d operator-(const Vector2d & other) const
  {
    return Vector2d(x_ - other.x_, y_ - other.y_);
  }
  Vector2d operator*(double scalar) const { return Vector2d(x_ * scalar, y_ * scalar); }
  double dot(const Vector2d & other) const { return x_ * other.x_ + y_ * other.y_; }
  double norm() const { return std::sqrt(x_ * x_ + y_ * y_); }
  Vector2d normalized() const
  {
    double n = norm();
    if (n > 0) {
      return Vector2d(x_ / n, y_ / n);
    }
    return Vector2d(0, 0);  // Or throw an exception for zero vector
  }

  // Overload for ostream to print Vector2d
  friend std::ostream & operator<<(std::ostream & os, const Vector2d & vec)
  {
    os << "(" << vec.x_ << ", " << vec.y_ << ")";  // Use private members
    return os;
  }
};
}  // namespace crane
#endif  // CRANE_BASICS__VECTOR2D_HPP_
