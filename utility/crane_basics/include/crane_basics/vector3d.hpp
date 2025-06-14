// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__VECTOR3D_HPP_
#define CRANE_BASICS__VECTOR3D_HPP_

#include <cmath>
#include <iostream>

namespace crane
{

class Vector3d
{
private:
  double x_, y_, z_;

public:
  // Helper class for comma initialization
  class CommaInitializer
  {
  public:
    CommaInitializer(Vector3d & vec, double val_x) : vec_(vec), x_val_(val_x), y_set_(false) {}
    // clang-format off
    // Prevent formatter from breaking this onto multiple lines in a way that Clang-Tidy dislikes
    CommaInitializer& operator,(double val) {
      // clang-format on
      if (!y_set_) {
        vec_.x_ = x_val_;
        vec_.y_ = val;
        y_set_ = true;
        return *this;
      } else {
        vec_.z_ = val;
        return *this;
      }
    }

  private:
    Vector3d & vec_;
    double x_val_;
    bool y_set_;
  };

  // Constructors
  Vector3d() : x_(0.0), y_(0.0), z_(0.0) {}
  Vector3d(double x_val, double y_val, double z_val) : x_(x_val), y_(y_val), z_(z_val) {}
  Vector3d(double x_val, double y_val) : x_(x_val), y_(y_val), z_(0.0) {}

  // Accessors (Eigen-like)
  double x() const { return x_; }  // const getter
  double y() const { return y_; }  // const getter
  double z() const { return z_; }  // const getter

  double & x() { return x_; }  // non-const getter/setter
  double & y() { return y_; }  // non-const getter/setter
  double & z() { return z_; }  // non-const getter/setter

  // Initialization with << and comma
  CommaInitializer operator<<(double val_x) { return CommaInitializer(*this, val_x); }

  // Vector operations
  Vector3d & operator+=(const Vector3d & other)
  {
    x_ += other.x_;
    y_ += other.y_;
    z_ += other.z_;
    return *this;
  }
  Vector3d & operator-=(const Vector3d & other)
  {
    x_ -= other.x_;
    y_ -= other.y_;
    z_ -= other.z_;
    return *this;
  }
  Vector3d operator+(const Vector3d & other) const
  {
    return Vector3d(x_ + other.x_, y_ + other.y_, z_ + other.z_);
  }
  Vector3d operator-(const Vector3d & other) const
  {
    return Vector3d(x_ - other.x_, y_ - other.y_, z_ - other.z_);
  }

  Vector3d operator*(double scalar) const
  {
    return Vector3d(x_ * scalar, y_ * scalar, z_ * scalar);
  }
  // Friend declaration for scalar * vector
  friend Vector3d operator*(double scalar, const Vector3d & vec);

  Vector3d operator-() const { return Vector3d(-x_, -y_, -z_); }
  Vector3d operator/(double scalar) const
  {
    if (scalar == 0) {
      // Consider logging a warning or throwing an exception in a real scenario.
      return Vector3d(0, 0, 0);
    }
    return Vector3d(x_ / scalar, y_ / scalar, z_ / scalar);
  }

  double dot(const Vector3d & other) const { return x_ * other.x_ + y_ * other.y_ + z_ * other.z_; }
  double norm() const { return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_); }

  // squaredNorm method
  double squaredNorm() const { return x_ * x_ + y_ * y_ + z_ * z_; }

  // Static method for zero vector
  static Vector3d Zero() { return Vector3d(0.0, 0.0, 0.0); }

  // Comparison operators
  bool operator==(const Vector3d & other) const
  {
    // Add a small epsilon for floating point comparison if exact comparison is not desired
    // For now, doing direct comparison. Consider if epsilon is needed based on usage.
    return x_ == other.x_ && y_ == other.y_ && z_ == other.z_;
  }

  bool operator!=(const Vector3d & other) const { return !(*this == other); }

  Vector3d normalized() const
  {
    double n = norm();
    if (n > 0) {
      return Vector3d(x_ / n, y_ / n, z_ / n);
    }
    return Vector3d(0, 0, 0);
  }

  // In-place normalization
  void normalize()
  {
    double n = norm();  // norm() should already exist
    if (n > 0) {
      x_ /= n;
      y_ /= n;
      z_ /= n;
    } else {
      // Handle zero vector case, e.g., do nothing or set to a specific state
      // Eigen's normalize() for a zero vector results in a zero vector.
      // So, doing nothing (leaving x_, y_, and z_ as they are, likely 0) is consistent.
    }
  }

  // Cross product (3D specific)
  Vector3d cross(const Vector3d & other) const
  {
    return Vector3d(
      y_ * other.z_ - z_ * other.y_, z_ * other.x_ - x_ * other.z_, x_ * other.y_ - y_ * other.x_);
  }

  friend std::ostream & operator<<(std::ostream & os, const Vector3d & vec)
  {
    os << "(" << vec.x_ << ", " << vec.y_ << ", " << vec.z_ << ")";
    return os;
  }
};  // End of Vector3d class

// Definition of the friend operator
inline Vector3d operator*(double scalar, const Vector3d & vec)
{
  return Vector3d(
    scalar * vec.x_, scalar * vec.y_,
    scalar * vec.z_);  // Access private members directly as it's a friend
}

}  // namespace crane
#endif  // CRANE_BASICS__VECTOR3D_HPP_
