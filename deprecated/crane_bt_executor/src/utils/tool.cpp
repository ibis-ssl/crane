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

#include <experimental/optional>
#include <utility>

#include "crane_bt_executor/utils/tool.hpp"
#include "crane_geometry/boost_geometry.hpp"

Segment tool::getSegment(Point base, Point target)
{
  constexpr float LARGE_DISTANCE = 20.0f;
  Point norm = (target - base).normalized();
  return std::move(Segment(base, base + norm * LARGE_DISTANCE));
}

Point tool::getPoint(const geometry_msgs::msg::Pose2D& pose)
{
  Point vec;
  vec << pose.x, pose.y;
  return vec;
}

geometry_msgs::msg::Pose2D tool::getPose2D(const Point& vec)
{
  geometry_msgs::msg::Pose2D pose;
  pose.x = vec.x();
  pose.y = vec.y();
  pose.theta = 0.0f;
  return pose;
}

//  geometry_msgs::msg::Pose2D tool::getPose2D(const Pose2D pose)
//  {
//    geometry_msgs::msg::Pose2D geo_pose;
//    geo_pose.x = pose.pos.x();
//    geo_pose.y = pose.pos.y();
//    geo_pose.theta = pose.theta;
//    return geo_pose;
//  }

//  Pose2D tool::getPose2D(const geometry_msgs::msg::Pose2D geo_pose)
//  {
//    Pose2D pose;
//    pose.pos << geo_pose.x, geo_pose.y;
//    pose.theta = geo_pose.theta;
//    return pose;
//  }

Point tool::getPoint(const Eigen::Vector3f& vec3)
{
  Point vec2;
  vec2 << vec3.x(), vec3.y();
  return vec2;
}

float tool::getDeg(float angle_rad)
{
  return angle_rad * 180.0f * M_1_PI;
}

float tool::getRad(float angle_deg)
{
  return angle_deg * M_PI / 180.0f;
}

float tool::getAngle(Point vec)
{
  return atan2(vec.y(), vec.x());
}

/**
 * -pi~piの範囲にする
 */
float tool::normalizeAngle(float angle_rad)
{
  while (angle_rad > M_PI)
  {
    angle_rad -= 2.0f * M_PI;
  }
  while (angle_rad < -M_PI)
  {
    angle_rad += 2.0f * M_PI;
  }
  return angle_rad;
}

float tool::getAngleDiff(float angle_rad1, float angle_rad2)
{
  angle_rad1 = normalizeAngle(angle_rad1);
  angle_rad2 = normalizeAngle(angle_rad2);
  if (abs(angle_rad1 - angle_rad2) > M_PI)
  {
    return abs(angle_rad1 + angle_rad2);
  }
  else
  {
    return abs(angle_rad1 - angle_rad2);
  }
}

float tool::getIntermediateAngle(float angle_rad1, float angle_rad2)
{
  angle_rad1 = normalizeAngle(angle_rad1);
  angle_rad2 = normalizeAngle(angle_rad2);
  // 差がpiを超えている場合では平均を取るだけではダメ
  if (abs(angle_rad1 - angle_rad2) > M_PI)
  {
    return normalizeAngle((angle_rad1 + angle_rad2 + 2.0f * M_PI) / 2.0f);
  }
  else
  {
    return (angle_rad1 + angle_rad2) / 2.0f;
  }
}

Point tool::getVerticalVec(Point v)
{
  Point vertical_v;
  vertical_v << v.y(), -v.x();
  return vertical_v;
}

Point tool::getUnitVec(float theta)
{
  Point vec;
  vec << cos(theta), sin(theta);
  return vec;
}

Point tool::getVec(float length, float theta)
{
  return getUnitVec(theta) * length;
}

Point tool::getDirectonNorm(Point base, Point target)
{
  return (target - base).normalized();
}

Point tool::getDirectonVec(Point base, Point target)
{
  return target - base;
}

float tool::getReachTime(float distance, float v0, float acc, float max_vel)
{
  // x = v0*t + 1/2*a*t^2 より
  float t = (sqrt(v0 * v0 + 2.0f * acc * distance) - v0) / acc;
  if (max_vel == -1.f)
  {
    return t;
  }
  else
  {
    float acc_end_time = (max_vel - v0) / acc;
    if (t > acc_end_time)
    {
      return (distance + 0.5f * std::pow(max_vel - v0, 2.f) / acc) / max_vel;
    }
    else
    {
      return t;
    }
  }
}

//  Point tool::getBallLineClosestPoint(Point from_pos, const BallInfo & ball)
//  {
//    Segment ball_line(ball.pose.pos, (ball.pose.pos + ball.pose.vel.normalized() * 30.f));
//    ClosestPoint result;
//    bg::closest_point(from_pos, ball_line, result);
//    return result.closest_point;
//  }

//  bool tool::isInOurDefenseArea(Point pos)
//  {
//    Point uf = Constants::Our::Penalty::upper_front();
//    Point lf = Constants::Our::Penalty::lower_front();
//    if (pos.x() < uf.x() && pos.y() < uf.y() && pos.y() > lf.y()) {
//      return true;
//    }
//    return false;
//  }
//
//
//  bool tool::isInTheirDefenseArea(Point pos)
//  {
//    Point uf = Constants::Their::Penalty::upper_front();
//    Point lf = Constants::Their::Penalty::lower_front();
//    if (pos.x() > uf.x() && pos.y() < uf.y() && pos.y() > lf.y()) {
//      return true;
//    }
//    return false;
//  }
