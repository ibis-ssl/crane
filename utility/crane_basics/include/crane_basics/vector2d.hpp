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
  double x_, y_;

public:
  // Helper class for comma initialization
  class CommaInitializer
  {
  public:
    CommaInitializer(Vector2d & vec, double val_x) : vec_(vec), x_val_(val_x) {}
    // clang-format off
    // Prevent formatter from breaking this onto multiple lines in a way that Clang-Tidy dislikes
    Vector2d& operator,(double val_y) {
      // clang-format on
      vec_.x_ = x_val_;  // Directly access private members as CommaInitializer is a nested class
      vec_.y_ = val_y;
      return vec_;
    }

  private:
    Vector2d & vec_;
    double x_val_;
  };

  // Constructors
  Vector2d() : x_(0.0), y_(0.0) {}
  Vector2d(double x_val, double y_val) : x_(x_val), y_(y_val) {}

  // Accessors (Eigen-like)
  double x() const { return x_; }  // const getter
  double y() const { return y_; }  // const getter

  double & x() { return x_; }  // non-const getter/setter
  double & y() { return y_; }  // non-const getter/setter

  // Initialization with << and comma
  CommaInitializer operator<<(double val_x) { return CommaInitializer(*this, val_x); }

  // Vector operations
  Vector2d & operator+=(const Vector2d & other)
  {
    x_ += other.x_;
    y_ += other.y_;
    return *this;
  }
  Vector2d & operator-=(const Vector2d & other)
  {
    x_ -= other.x_;
    y_ -= other.y_;
    return *this;
  }
  Vector2d operator+(const Vector2d & other) const
  {
    return Vector2d(x_ + other.x_, y_ + other.y_);
  }
  Vector2d operator-(const Vector2d & other) const
  {
    return Vector2d(x_ - other.x_, y_ - other.y_);
  }

  Vector2d operator*(double scalar) const { return Vector2d(x_ * scalar, y_ * scalar); }
  // Friend declaration for scalar * vector
  friend Vector2d operator*(double scalar, const Vector2d & vec);

  Vector2d operator-() const { return Vector2d(-x_, -y_); }
  Vector2d operator/(double scalar) const
  {
    if (scalar == 0) {
      // Consider logging a warning or throwing an exception in a real scenario.
      return Vector2d(0, 0);
    }
    return Vector2d(x_ / scalar, y_ / scalar);
  }

  double dot(const Vector2d & other) const { return x_ * other.x_ + y_ * other.y_; }
  double norm() const { return std::sqrt(x_ * x_ + y_ * y_); }

  // squaredNorm method
  double squaredNorm() const { return x_ * x_ + y_ * y_; }

  // Static method for zero vector
  static Vector2d Zero() { return Vector2d(0.0, 0.0); }

  // Comparison operators
  bool operator==(const Vector2d & other) const
  {
    // Add a small epsilon for floating point comparison if exact comparison is not desired
    // For now, doing direct comparison. Consider if epsilon is needed based on usage.
    return x_ == other.x_ && y_ == other.y_;
  }

  bool operator!=(const Vector2d & other) const { return !(*this == other); }

  Vector2d normalized() const
  {
    double n = norm();
    if (n > 0) {
      return Vector2d(x_ / n, y_ / n);
    }
    return Vector2d(0, 0);
  }

  // In-place normalization
  void normalize()
  {
    double n = norm();  // norm() should already exist
    if (n > 0) {
      x_ /= n;
      y_ /= n;
    } else {
      // Handle zero vector case, e.g., do nothing or set to a specific state
      // Eigen's normalize() for a zero vector results in a zero vector.
      // So, doing nothing (leaving x_ and y_ as they are, likely 0) is consistent.
    }
  }

  friend std::ostream & operator<<(std::ostream & os, const Vector2d & vec)
  {
    os << "(" << vec.x_ << ", " << vec.y_ << ")";
    return os;
  }
};  // End of Vector2d class

// Definition of the friend operator
inline Vector2d operator*(double scalar, const Vector2d & vec)
{
  return Vector2d(
    scalar * vec.x_, scalar * vec.y_);  // Access private members directly as it's a friend
}

}  // namespace crane
#endif  // CRANE_BASICS__VECTOR2D_HPP_
