// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__DDPS_HPP_
#define CRANE_BASICS__DDPS_HPP_

#include <algorithm>
#include <cmath>
#include <crane_geometry/boost_geometry.hpp>
#include <vector>

namespace crane
{
inline auto getDPPSPoints(
  const Point & center, double r_resolution, double r_max, int theta_div_num) -> std::vector<Point>
{
  std::vector<Point> points;
  for (int theta_index = 0; theta_index < theta_div_num; theta_index++) {
    double theta = 2.0 * M_PI * theta_index / theta_div_num;
    for (double r = r_resolution; r <= r_max; r += r_resolution) {
      points.emplace_back(Point(center.x() + r * cos(theta), center.y() + r * sin(theta)));
    }
  }
  return points;
}

inline auto getPoints(const Segment & ball_line, double interval) -> std::vector<Point>
{
  std::vector<Point> points;
  float ball_line_len = (ball_line.first - ball_line.second).norm();
  auto norm_vec = (ball_line.second - ball_line.first).normalized();
  for (double d = 0.0; d <= ball_line_len; d += interval) {
    points.emplace_back(ball_line.first + d * norm_vec);
  }
  return points;
}

inline auto getPoints(const Point & center, float unit, int unit_num) -> std::vector<Point>
{
  std::vector<Point> points;
  for (float x = center.x() - unit * (unit_num / 2.f); x <= center.x() + unit * (unit_num / 2.f);
       x += unit) {
    for (float y = center.y() - unit * (unit_num / 2.f); y <= center.y() + unit * (unit_num / 2.f);
         y += unit) {
      points.emplace_back(Point(x, y));
    }
  }
  return points;
}

inline auto getPoints(
  const Point & center, float unit_x, float unit_y, int unit_num_x, int unit_num_y)
  -> std::vector<Point>
{
  std::vector<Point> points;
  for (float x = center.x() - unit_x * (unit_num_x / 2.f);
       x <= center.x() + unit_x * (unit_num_x / 2.f); x += unit_x) {
    for (float y = center.y() - unit_y * (unit_num_y / 2.f);
         y <= center.y() + unit_y * (unit_num_y / 2.f); y += unit_y) {
      points.emplace_back(Point(x, y));
    }
  }
  return points;
}

}  // namespace crane
#endif  // CRANE_BASICS__DDPS_HPP_
