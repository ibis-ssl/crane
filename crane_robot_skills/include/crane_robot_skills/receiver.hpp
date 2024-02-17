// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__RECEIVER_HPP_
#define CRANE_ROBOT_SKILLS__RECEIVER_HPP_

#include <crane_game_analyzer/evaluations/evaluations.hpp>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_robot_skills/skill_base.hpp>

namespace crane::skills
{
class Receiver : public SkillBase<>
{
public:
  enum class ReceivePhase {
    NONE,
    MOVE_ROUGH,
    MOVE_TO_EXPECTED_BALL_LINE,
    MOVE_TO_ACTUAL_BALL_LINE,
  };
  struct SessionInfo
  {
    int receiver_id;
  } session_info;

  struct PositionsWithScore
  {
    Point passer_pos;

    Point receiver_pos;

    double score;
  };

  explicit Receiver(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model);

  void print(std::ostream & os) const override { os << "[Receiver]"; }

  auto getLargestGoalAngleWidthFromPosition(const Point point) -> double
  {
    Interval goal_range;

    auto goal_posts = world_model->getTheirGoalPosts();
    goal_range.append(getAngle(goal_posts.first - point), getAngle(goal_posts.second - point));

    for (auto & enemy : world_model->theirs.robots) {
      double distance = (point - enemy->pose.pos).norm();
      constexpr double MACHINE_RADIUS = 0.1;

      double center_angle = getAngle(enemy->pose.pos - point);
      double diff_angle =
        atan(MACHINE_RADIUS / std::sqrt(distance * distance - MACHINE_RADIUS * MACHINE_RADIUS));

      goal_range.erase(center_angle - diff_angle, center_angle + diff_angle);
    }

    auto largest_interval = goal_range.getLargestInterval();
    return largest_interval.second - largest_interval.first;
  }

  auto getLargestGoalAngleFromPosition(const Point point) -> double
  {
    Interval goal_range;

    auto goal_posts = world_model->getTheirGoalPosts();
    goal_range.append(getAngle(goal_posts.first - point), getAngle(goal_posts.second - point));

    for (auto & enemy : world_model->theirs.robots) {
      double distance = (point - enemy->pose.pos).norm();
      constexpr double MACHINE_RADIUS = 0.1;

      double center_angle = getAngle(enemy->pose.pos - point);
      double diff_angle =
        atan(MACHINE_RADIUS / std::sqrt(distance * distance - MACHINE_RADIUS * MACHINE_RADIUS));

      goal_range.erase(center_angle - diff_angle, center_angle + diff_angle);
    }

    auto largest_interval = goal_range.getLargestInterval();
    return (largest_interval.second + largest_interval.first) * 0.5;
  }

  std::vector<std::pair<double, Point>> getPositionsWithScore(Segment ball_line, Point next_target)
  {
    auto points = getPoints(ball_line, 0.05);
    std::vector<std::pair<double, Point>> position_with_score;
    for (auto point : points) {
      double score = getPointScore(point, next_target);
      position_with_score.push_back(std::make_pair(score, point));
    }
    return position_with_score;
  }

  std::vector<Point> getPoints(Segment ball_line, double interval)
  {
    std::vector<Point> points;
    float ball_line_len = (ball_line.first - ball_line.second).norm();
    auto norm_vec = (ball_line.second - ball_line.first).normalized();
    for (double d = 0.0; d <= ball_line_len; d += interval) {
      points.emplace_back(ball_line.first + d * norm_vec);
    }
    return points;
  }

  std::vector<Point> getPoints(Point center, float unit, int unit_num)
  {
    std::vector<Point> points;
    for (float x = center.x() - unit * (unit_num / 2.f); x <= center.x() + unit * (unit_num / 2.f);
         x += unit) {
      for (float y = center.y() - unit * (unit_num / 2.f);
           y <= center.y() + unit * (unit_num / 2.f); y += unit) {
        points.emplace_back(Point(x, y));
      }
    }
    return points;
  }

  std::vector<Point> getDPPSPoints(Point center, double r_resolution, int theta_div_num)
  {
    std::vector<Point> points;
    for (int theta_index = 0; theta_index < theta_div_num; theta_index++) {
      double theta = 2.0 * M_PI * theta_index / theta_div_num;
      for (double r = r_resolution; r <= 10.0; r += r_resolution) {
        points.emplace_back(Point(center.x() + r * cos(theta), center.y() + r * sin(theta)));
      }
    }
    points.erase(
      std::remove_if(
        points.begin(), points.end(),
        [&](const auto & point) {
          return (not world_model->isFieldInside(point)) or world_model->isDefenseArea(point);
        }),
      points.end());

    return points;
  }

  double getPointScore(Point p, Point next_target)
  {
    double nearest_dist;
    RobotIdentifier receiver{true, static_cast<uint8_t>(session_info.receiver_id)};
    return evaluation::getNextTargetVisibleScore(p, next_target, world_model) *
           evaluation::getReachScore(receiver, p, nearest_dist, world_model) *
           evaluation::getAngleScore(receiver, p, next_target, world_model) *
           evaluation::getEnemyDistanceScore(p, world_model);
  }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__RECEIVER_HPP_
