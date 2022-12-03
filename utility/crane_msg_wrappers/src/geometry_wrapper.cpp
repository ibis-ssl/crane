// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/geometry_wrapper.hpp"

#include <cmath>

#include "rclcpp/clock.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace geometry2d
{
Pose::Pose()
{
  this->x = 0.0;
  this->y = 0.0;
  this->theta = 0.0;
}

Pose::Pose(float x, float y, float theta)
{
  this->x = x;
  this->y = y;
  this->theta = theta;
}

Pose::Pose(geometry_msgs::msg::Pose pose)
{
  this->x = pose.position.x;
  this->y = pose.position.y;
  this->theta = getYawFromQuaternion(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  this->theta = pi2pi(this->theta);
}

Pose::Pose(geometry_msgs::msg::Pose2D pose)
{
  this->x = pose.x;
  this->y = pose.y;
  this->theta = pose.theta;
  this->theta = pi2pi(this->theta);
}

MatrixWrapper::ColumnVector Pose::toColumnVector()
{
  MatrixWrapper::ColumnVector column_vector(3);

  column_vector(1) = this->x;
  column_vector(2) = this->y;
  column_vector(3) = this->theta;

  return column_vector;
}

geometry_msgs::msg::Pose Pose::toROSPose()
{
  geometry_msgs::msg::Pose msg;

  msg.position.x = this->x;
  msg.position.y = this->y;
  msg.orientation = getQuaternionFromYaw(this->theta);

  return msg;
}

geometry_msgs::msg::Pose2D Pose::toROSPose2D()
{
  geometry_msgs::msg::Pose2D msg;

  msg.x = this->x;
  msg.y = this->y;
  msg.theta = this->theta;

  return msg;
}

// geometry2d::Pose Pose::Transpose(geometry2d::Pose pose)
// このインスタンスの姿勢からみた pose の姿勢を返す
geometry2d::Pose Pose::transpose(geometry2d::Pose pose)
{
  geometry2d::Pose retval;

  float diff_x = pose.x - this->x;
  float diff_y = pose.y - this->y;

  retval.x = cos(this->theta) * diff_x - sin(this->theta) * diff_y;
  retval.y = sin(this->theta) * diff_x + cos(this->theta) * diff_y;
  retval.theta = pose.theta - this->theta;

  return retval;
}

// float Pose::GetNorm()
// 位置のノルム（距離）を返す
float Pose::getNorm() { return sqrt(pow(this->x, 2) + pow(this->y, 2)); }

// float Pose::getAngle()
// 原点からみた自身のなす角を返す（極座標表現での角度）
float Pose::getAngle() { return atan2(this->y, this->x); }

Velocity::Velocity()
{
  this->x = 0.0;
  this->y = 0.0;
  this->theta = 0.0;
}

Velocity::Velocity(float x, float y, float theta)
{
  this->x = x;
  this->y = y;
  this->theta = theta;
}

Velocity::Velocity(geometry_msgs::msg::Twist twist)
{
  this->x = twist.linear.x;
  this->y = twist.linear.y;
  this->theta = twist.angular.z;
}

geometry_msgs::msg::Twist Velocity::toROSTwist()
{
  geometry_msgs::msg::Twist msg;

  msg.linear.x = this->x;
  msg.linear.y = this->y;
  msg.angular.z = this->theta;

  return msg;
}

// float Velocity::GetNorm()
// 速度ベクトルのノルムを返す
float Velocity::getNorm() { return sqrt(pow(this->x, 2) + pow(this->y, 2)); }

// float Velocity::getAngle()
// 速度ベクトルが原点となす角度を返す
float Velocity::getAngle() { return atan2(this->y, this->x); }

Accel::Accel()
{
  this->x = 0.0;
  this->y = 0.0;
  this->theta = 0.0;
}

Accel::Accel(float x, float y, float theta)
{
  this->x = x;
  this->y = y;
  this->theta = theta;
}

Accel::Accel(geometry_msgs::msg::Accel accel)
{
  this->x = accel.linear.x;
  this->y = accel.linear.y;
  this->theta = accel.angular.z;
}

MatrixWrapper::ColumnVector Accel::toColumnVector()
{
  MatrixWrapper::ColumnVector column_vector(3);

  column_vector(1) = this->x;
  column_vector(2) = this->y;
  column_vector(3) = this->theta;

  return column_vector;
}

Odometry::Odometry() {}

Odometry::Odometry(Pose pose, Velocity velocity)
{
  this->pose = pose;
  this->velocity = velocity;
}

nav_msgs::msg::Odometry Odometry::toROSOdometry()
{
  nav_msgs::msg::Odometry msg;

  msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  msg.header.frame_id = "map";  // TODO(HansRobo): パラメータ化

  msg.pose.pose = this->pose.toROSPose();
  msg.twist.twist = this->velocity.toROSTwist();

  return msg;
}

void Odometry::print()
{
  std::cout << "[Odometry print]" << std::endl;
  std::cout << "[Pose] x:" << this->pose.x << ", y:" << this->pose.y
            << ", theta:" << this->pose.theta << std::endl;
  std::cout << "[Velo] x:" << this->velocity.x << ", y:" << this->velocity.y
            << ", theta:" << this->velocity.theta << std::endl;
}

float getYawFromQuaternion(float x, float y, float z, float w)
{
  double roll, pitch, yaw;
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion getQuaternionFromYaw(float theta)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);

  return tf2::toMsg(q);
}

float pi2pi(float rad)
{
  while (rad >= M_PI) {
    rad -= 2.0 * M_PI;
  }
  while (rad <= -M_PI) {
    rad += 2.0 * M_PI;
  }
  return rad;
}

}  // namespace geometry2d
