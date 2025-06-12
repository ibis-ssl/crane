// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__ROTATION2D_HPP_
#define CRANE_BASICS__ROTATION2D_HPP_

#include <cmath>
#include "vector2d.hpp" // Assuming vector2d.hpp is in the same directory

namespace crane
{

class Rotation2D
{
public:
  // Default constructor (identity rotation)
  Rotation2D() : angle_rad_(0.0), cos_angle_(1.0), sin_angle_(0.0) {}

  // Constructor from angle in radians
  explicit Rotation2D(double angle_rad) : angle_rad_(angle_rad) {
    cos_angle_ = std::cos(angle_rad_);
    sin_angle_ = std::sin(angle_rad_);
  }

  // Get the rotation angle in radians
  double angle() const { return angle_rad_; }

  // Apply rotation to a Vector2d
  // x' = x * cos(angle) - y * sin(angle)
  // y' = x * sin(angle) + y * cos(angle)
  Vector2d operator*(const Vector2d& vec) const {
    double new_x = vec.x() * cos_angle_ - vec.y() * sin_angle_;
    double new_y = vec.x() * sin_angle_ + vec.y() * cos_angle_;
    return Vector2d(new_x, new_y);
  }

  // Could add other helpful methods like:
  // - fromMatrix (if we had a Matrix2d class)
  // - inverse()
  // - slerp (spherical linear interpolation) with another Rotation2D

private:
  double angle_rad_;
  double cos_angle_;
  double sin_angle_;
};

}  // namespace crane

#endif  // CRANE_BASICS__ROTATION2D_HPP_
