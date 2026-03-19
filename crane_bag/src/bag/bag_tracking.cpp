// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_tracking.hpp"

#include <cmath>
#include <limits>

namespace crane::bag
{

std::vector<RobotState> track_robot(
  const BagData & data, int robot_id, bool is_ours, double interval,
  std::optional<std::pair<double, double>> time_range)
{
  std::vector<RobotState> result;
  int64_t bag_start = data.info.start_time_ns;
  auto [filter_start, filter_end] = make_ns_range(bag_start, time_range);
  int64_t interval_ns = static_cast<int64_t>(interval * 1e9);
  int64_t last_ns = 0;

  for (const auto & tm : data.world_models) {
    if (tm.timestamp_ns < filter_start || tm.timestamp_ns > filter_end) continue;
    if (tm.timestamp_ns - last_ns < interval_ns) continue;
    last_ns = tm.timestamp_ns;  // ロボット有無に関わらずインターバルを進める

    const auto & msg = tm.msg;
    const auto & ball = msg.ball_info;
    double bx = ball.position.x, by = ball.position.y;

    const auto & robots = is_ours ? msg.robot_info_ours : msg.robot_info_theirs;
    for (const auto & r : robots) {
      if (static_cast<int>(r.id) == robot_id) {
        double vx = r.velocity.x, vy = r.velocity.y;
        RobotState s;
        s.t = tm.t(bag_start);
        s.robot_id = robot_id;
        s.x = r.pose.x;
        s.y = r.pose.y;
        s.theta = r.pose.theta;
        s.vx = vx;
        s.vy = vy;
        s.speed = std::sqrt(vx * vx + vy * vy);
        s.detected = r.available_vision;
        s.dist_to_ball =
          std::sqrt((r.pose.x - bx) * (r.pose.x - bx) + (r.pose.y - by) * (r.pose.y - by));
        result.push_back(s);
        break;
      }
    }
  }
  return result;
}

std::vector<BallState> track_ball(
  const BagData & data, double interval, std::optional<std::pair<double, double>> time_range)
{
  std::vector<BallState> result;
  int64_t bag_start = data.info.start_time_ns;
  auto [filter_start, filter_end] = make_ns_range(bag_start, time_range);
  int64_t interval_ns = static_cast<int64_t>(interval * 1e9);
  int64_t last_ns = 0;

  for (const auto & tm : data.world_models) {
    if (tm.timestamp_ns < filter_start || tm.timestamp_ns > filter_end) continue;
    if (tm.timestamp_ns - last_ns < interval_ns) continue;

    const auto & ball = tm.msg.ball_info;
    double vx = ball.velocity.x, vy = ball.velocity.y;
    BallState s;
    s.t = tm.t(bag_start);
    s.x = ball.position.x;
    s.y = ball.position.y;
    s.vx = vx;
    s.vy = vy;
    s.speed = std::sqrt(vx * vx + vy * vy);
    result.push_back(s);
    last_ns = tm.timestamp_ns;
  }
  return result;
}

}  // namespace crane::bag
