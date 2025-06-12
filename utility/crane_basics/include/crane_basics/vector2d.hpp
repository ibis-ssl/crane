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
  double x_, y_; // Changed to private and named with underscore
  bool is_setting_x_ = true; // Helper state for << operator

public:
  // Constructors
  Vector2d() : x_(0.0), y_(0.0), is_setting_x_(true) {}
  Vector2d(double x_val, double y_val) : x_(x_val), y_(y_val), is_setting_x_(true) {}

  // Accessors (Eigen-like)
  double x() const { return x_; } // const getter
  double y() const { return y_; } // const getter

  double& x() { return x_; }      // non-const getter/setter
  double& y() { return y_; }      // non-const getter/setter

  // Vector operations
  Vector2d operator+(const Vector2d & other) const { return Vector2d(x_ + other.x_, y_ + other.y_); }
  Vector2d operator-(const Vector2d & other) const { return Vector2d(x_ - other.x_, y_ - other.y_); }
  Vector2d operator*(double scalar) const { return Vector2d(x_ * scalar, y_ * scalar); }

  // Compound assignment operators
  Vector2d& operator+=(const Vector2d & other) {
    x_ += other.x_;
    y_ += other.y_;
    return *this;
  }

  Vector2d& operator-=(const Vector2d & other) {
    x_ -= other.x_;
    y_ -= other.y_;
    return *this;
  }

  // Unary minus operator
  Vector2d operator-() const {
    return Vector2d(-x_, -y_);
  }

  // Scalar division operator
  Vector2d operator/(double scalar) const {
    if (scalar == 0) {
      // Or throw an exception, or return a zero vector, depending on desired behavior
      // For now, returning a zero vector to avoid division by zero errors silently.
      // Consider logging a warning or throwing an exception in a real scenario.
      return Vector2d(0,0);
    }
    return Vector2d(x_ / scalar, y_ / scalar);
  }

  double dot(const Vector2d & other) const { return x_ * other.x_ + y_ * other.y_; }
  double norm() const { return std::sqrt(x_ * x_ + y_ * y_); }
  Vector2d normalized() const
  {
    double n = norm();
    if (n > 0) {
      return Vector2d(x_ / n, y_ / n);
    }
    return Vector2d(0, 0); // Or throw an exception for zero vector
  }

  // Overload for ostream to print Vector2d
  friend std::ostream & operator<<(std::ostream & os, const Vector2d & vec)
  {
    os << "(" << vec.x_ << ", " << vec.y_ << ")"; // Use private members
    return os;
  }

  // Overload for << initialization (for 'vec << val1 << val2;')
  Vector2d& operator<<(double value) {
    if (is_setting_x_) {
      x_ = value;
      is_setting_x_ = false;
    } else {
      y_ = value;
      is_setting_x_ = true; // Reset for next potential initialization
    }
    return *this;
  }
};
}  // namespace crane
#endif  // CRANE_BASICS__VECTOR2D_HPP_
