// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/geometry_operations.hpp>
#include <crane_robot_skills/kick.hpp>
#include <magic_enum/magic_enum.hpp>

#include "../include/crane_robot_skills/single_ball_placement.hpp"

namespace crane::skills
{
std::string Kick::getStateName(int s)
{
  return std::string(magic_enum::enum_name(static_cast<KickState>(s)));
}

void Kick::initialize()
{
  setParameter("target", Point(0, 0));
  setParameter("kick_power", 0.7f);
  setParameter("use_target_kick_speed", false);
  setParameter("target_kick_speed", 2.0);
  setParameter("use_target_chip_distance", false);
  setParameter("target_chip_distance", 2.0);
  setParameter("chip_kick", false);
  setParameter("with_dribble", false);
  setParameter("dribble_power", 0.3f);
  setParameter("angle_threshold_deg", 15.0f);
  setParameter("around_interval", 0.15f);
  setParameter("go_around_ball", true);
  setParameter("kicked_speed_threshold", 1.5);

  addStateFunction(static_cast<int>(KickState::ENTRY_POINT), [this]() {
    visualizer->drawDebugLabel(robot()->pose.pos, "Kick::ENTRY_POINT");
    return Status::RUNNING;
  });

  addTransition(
    static_cast<int>(KickState::ENTRY_POINT), static_cast<int>(KickState::AROUND_BALL_AND_KICK),
    [this]() { return true; });

  addStateFunction(static_cast<int>(KickState::AROUND_BALL_AND_KICK), [this]() {
    auto target = getParameter<Point>("target");
    Point ball_pos = world_model()->ball().pos;
    const double interval = std::max(getParameter<double>("around_interval"), 0.01);
    const bool go_around_ball = getParameter<bool>("go_around_ball");
    const auto kick_vec = [&]() -> Vector2 {
      auto target_vec = target - ball_pos;
      if (target_vec.norm() > 1e-6) {
        return target_vec.normalized();
      }
      auto fallback = robot()->pose.pos - ball_pos;
      if (fallback.norm() > 1e-6) {
        return fallback.normalized();
      }
      return Vector2(1.0, 0.0);
    }();
    Point safe_target = ball_pos + kick_vec;

    // 視認性の高いキック方向の可視化: 太い矢印 + 角度しきい値の扇
    {
      Vector2 dir = kick_vec;
      // 長めの矢印（フィールド半分+余裕）
      double arrow_len = world_model()->fieldSize().x() * 0.5 + 0.5;

      // メインの矢印シャフト + アローヘッド
      visualizer->arrow(ball_pos, dir, arrow_len, "lime", 20, 0.35, 0.20);

      // 角度しきい値の扇（境界線 + アーク）
      using boost::math::constants::degree;
      double half_angle = getParameter<double>("angle_threshold_deg") * degree<double>();
      double base_theta = getAngle(dir);
      double arc_radius = 0.9;
      // 境界線
      auto dir_left = Vector2(std::cos(base_theta + half_angle), std::sin(base_theta + half_angle));
      auto dir_right =
        Vector2(std::cos(base_theta - half_angle), std::sin(base_theta - half_angle));
      visualizer->drawLine(ball_pos, ball_pos + dir_left * arc_radius, "white", 10, 0.6);
      visualizer->drawLine(ball_pos, ball_pos + dir_right * arc_radius, "white", 10, 0.6);
      // アーク（扇の円弧）
      visualizer->arc(
        ball_pos, arc_radius, base_theta - half_angle, base_theta + half_angle, "white", 10, 16);
    }
    visualizer->drawDebugLabel(robot()->pose.pos, "Kick::AROUND_BALL");
    Point approach = [&]() -> Point {
      if (!go_around_ball) {
        return ball_pos - kick_vec * interval;
      }
      // 改良回り込み: ロボット->目標線分に対するボール最近傍方向へ回り込み
      double max_interval = std::max(interval, interval * 2.0);
      return computeAroundBallApproachTargetDynamic(
        ball_pos, safe_target, robot()->pose.pos, interval, max_interval);
    }();

    double kick_vec_gain = [&]() {
      Segment ball_kick_zone{ball_pos, ball_pos - kick_vec * interval};
      if (bg::distance(ball_kick_zone, robot()->pose.pos) < 0.1) {
        command->disableCollisionAvoidance();
        return 0.5;
      } else {
        return 0.0;
      }
    }();

    command->lookAtFrom(safe_target, ball_pos)
      .setDribblerTargetPosition(approach + kick_vec * kick_vec_gain);
    command->disableBallAvoidance();
    using boost::math::constants::degree;
    const double angle_threshold = getParameter<double>("angle_threshold_deg") * degree<double>();
    if (
      std::abs(getAngleDiff(getAngle(kick_vec), getAngle(ball_pos - robot()->pose.pos))) <
      angle_threshold) {
      if (getParameter<bool>("chip_kick")) {
        kickWithChip();
      } else {
        kickStraight();
      }
    } else {
      command->kickStraight(0.0);
    }

    if (getParameter<bool>("with_dribble")) {
      command->withDribble(getParameter<double>("dribble_power"));
    } else {
      // ドリブラーを止める
      command->withDribble(0.0);
    }
    return Status::RUNNING;
  });

  addTransition(
    static_cast<int>(KickState::AROUND_BALL_AND_KICK), static_cast<int>(KickState::ENTRY_POINT),
    [this]() {
      // 素早く遠ざかっていったら終了
      return world_model()->ball().isMoving(getParameter<double>("kicked_speed_threshold")) &&
             world_model()->ball().isMovingAwayFrom(robot()->pose.pos, 30.);
    });
}

auto Kick::getBallExitPointFromField(const double offset) -> Point
{
  Segment ball_line{
    world_model()->ball().pos,
    world_model()->ball().pos + world_model()->ball().vel.normalized() * 10.0};

  const double X = world_model()->fieldSize().x() / 2.0 - offset;
  const double Y = world_model()->fieldSize().y() / 2.0 - offset;

  std::vector<Segment> segments;
  segments.emplace_back(Point(X, Y), Point(X, -Y));
  segments.emplace_back(Point(-X, Y), Point(-X, -Y));
  segments.emplace_back(Point(X, Y), Point(-X, Y));
  segments.emplace_back(Point(X, -Y), Point(-X, -Y));

  for (const auto & seg : segments) {
    if (auto intersections = getIntersections(ball_line, seg); not intersections.empty()) {
      return intersections.front();
    }
  }
  return world_model()->ball().pos;
}

auto Kick::kickWithChip() -> void
{
  if (getParameter<bool>("use_target_chip_distance")) {
    command->setKickWithChipTargetDistance(getParameter<double>("target_chip_distance"));
  } else {
    command->kickWithChip(getParameter<double>("kick_power"));
  }
}

auto Kick::kickStraight() -> void
{
  if (getParameter<bool>("use_target_kick_speed")) {
    command->setKickStraightTargetSpeed(getParameter<double>("target_kick_speed"));
  } else {
    command->kickStraight(getParameter<double>("kick_power"));
  }
}
}  // namespace crane::skills
