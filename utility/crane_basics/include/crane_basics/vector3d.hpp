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
  // コンストラクタ
  Vector3d() : x_(0.0), y_(0.0), z_(0.0) {}
  Vector3d(double x, double y, double z) : x_(x), y_(y), z_(z) {}

  // アクセサー
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

  double & x() { return x_; }
  double & y() { return y_; }
  double & z() { return z_; }

  // 基本演算子
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

  Vector3d operator/(double scalar) const
  {
    if (std::abs(scalar) < 1e-10) {
      throw std::runtime_error("Division by zero in Vector3d");
    }
    return Vector3d(x_ / scalar, y_ / scalar, z_ / scalar);
  }

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

  Vector3d & operator*=(double scalar)
  {
    x_ *= scalar;
    y_ *= scalar;
    z_ *= scalar;
    return *this;
  }

  Vector3d & operator/=(double scalar)
  {
    if (std::abs(scalar) < 1e-10) {
      throw std::runtime_error("Division by zero in Vector3d");
    }
    x_ /= scalar;
    y_ /= scalar;
    z_ /= scalar;
    return *this;
  }

  // 比較演算子
  bool operator==(const Vector3d & other) const
  {
    constexpr double epsilon = 1e-9;
    return std::abs(x_ - other.x_) < epsilon && 
           std::abs(y_ - other.y_) < epsilon && 
           std::abs(z_ - other.z_) < epsilon;
  }

  bool operator!=(const Vector3d & other) const
  {
    return !(*this == other);
  }

  // ベクトル演算
  double norm() const
  {
    return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
  }

  double squaredNorm() const
  {
    return x_ * x_ + y_ * y_ + z_ * z_;
  }

  Vector3d normalized() const
  {
    double n = norm();
    if (n < 1e-10) {
      return Vector3d(0, 0, 0);
    }
    return *this / n;
  }

  void normalize()
  {
    double n = norm();
    if (n > 1e-10) {
      *this /= n;
    } else {
      x_ = y_ = z_ = 0.0;
    }
  }

  double dot(const Vector3d & other) const
  {
    return x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
  }

  Vector3d cross(const Vector3d & other) const
  {
    return Vector3d(
      y_ * other.z_ - z_ * other.y_,
      z_ * other.x_ - x_ * other.z_,
      x_ * other.y_ - y_ * other.x_
    );
  }

  // 静的メソッド
  static Vector3d Zero()
  {
    return Vector3d(0, 0, 0);
  }

  static Vector3d UnitX()
  {
    return Vector3d(1, 0, 0);
  }

  static Vector3d UnitY()
  {
    return Vector3d(0, 1, 0);
  }

  static Vector3d UnitZ()
  {
    return Vector3d(0, 0, 1);
  }
};

// 外部演算子
inline Vector3d operator*(double scalar, const Vector3d & vec)
{
  return vec * scalar;
}

// ストリーム出力
inline std::ostream & operator<<(std::ostream & os, const Vector3d & vec)
{
  os << "(" << vec.x() << ", " << vec.y() << ", " << vec.z() << ")";
  return os;
}

}  // namespace crane

#endif  // CRANE_BASICS__VECTOR3D_HPP_