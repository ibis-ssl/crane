#ifndef GEOMETRY__GEOMETRY_HPP_
#define GEOMETRY__GEOMETRY_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <bfl/filter/extendedkalmanfilter.h>

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

#endif  // GEOMETRY__GEOMETRY_HPP_
