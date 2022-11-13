// Copyright (c) 2022 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_MSG_WRAPPERS__GEOMETRY_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__GEOMETRY_WRAPPER_HPP_

#include "bfl/filter/extendedkalmanfilter.h"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace geometry2d
{
//
// 単純型
//

// Poseクラス
// 2次元の姿勢(x, y, theta) を表現します。
class Pose
{
public:
  float x, y, theta;

  Pose();
  Pose(float x, float y, float theta);
  explicit Pose(geometry_msgs::msg::Pose pose);
  explicit Pose(geometry_msgs::msg::Pose2D pose);

  MatrixWrapper::ColumnVector toColumnVector();
  geometry_msgs::msg::Pose toROSPose();
  geometry_msgs::msg::Pose2D toROSPose2D();
  geometry2d::Pose transpose(geometry2d::Pose pose);

  // TODO(HansRobo): この辺の関数は Pose()クラスにあるべき
  float getNorm();
  float getAngle();
};

class Velocity
{
public:
  float x, y, theta;

  Velocity();
  Velocity(float x, float y, float theta);
  explicit Velocity(geometry_msgs::msg::Twist twist);

  geometry_msgs::msg::Twist toROSTwist();
  float getNorm();
  float getAngle();
};

class Accel
{
public:
  float x, y, theta;

  Accel();
  Accel(float x, float y, float theta);
  explicit Accel(geometry_msgs::msg::Accel accel);

  MatrixWrapper::ColumnVector toColumnVector();
};

class Point
{
public:
  float x, y;
};

//
// 複合型
//
class Odometry
{
public:
  Pose pose;
  Velocity velocity;

  Odometry();
  Odometry(Pose pose, Velocity velocity);

  nav_msgs::msg::Odometry toROSOdometry();

  void print();
};

// Utility methods
float getYawFromQuaternion(float x, float y, float z, float w);
geometry_msgs::msg::Quaternion getQuaternionFromYaw(float theta);
float pi2pi(float rad);
}  // namespace geometry2d

#endif  // CRANE_MSG_WRAPPERS__GEOMETRY_WRAPPER_HPP_
