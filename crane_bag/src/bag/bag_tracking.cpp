// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "bag_tracking.hpp"

#include <cmath>

namespace crane::bag
{

std::vector<RobotState> track_robot(
  const BagData & data, int robot_id, bool is_ours, double interval)
{
  std::vector<RobotState> result;
  int64_t bag_start = data.info.start_time_ns;

  // 間引きはロボットの有無に関わらず進める
  for (const auto * tm : BagData::sample(data.world_models, interval)) {
    const auto & msg = tm->msg;
    const auto & ball = msg.ball_info;
    double bx = ball.position.x, by = ball.position.y;

    const auto & robots = is_ours ? msg.robot_info_ours : msg.robot_info_theirs;
    for (const auto & r : robots) {
      if (static_cast<int>(r.id) == robot_id) {
        double vx = r.velocity.x, vy = r.velocity.y;
        RobotState s;
        s.t = tm->t(bag_start);
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

std::vector<BallState> track_ball(const BagData & data, double interval)
{
  std::vector<BallState> result;
  int64_t bag_start = data.info.start_time_ns;

  for (const auto * tm : BagData::sample(data.world_models, interval)) {
    const auto & ball = tm->msg.ball_info;
    double vx = ball.velocity.x, vy = ball.velocity.y;
    BallState s;
    s.t = tm->t(bag_start);
    s.x = ball.position.x;
    s.y = ball.position.y;
    s.vx = vx;
    s.vy = vy;
    s.speed = std::sqrt(vx * vx + vy * vy);
    result.push_back(s);
  }
  return result;
}

}  // namespace crane::bag
