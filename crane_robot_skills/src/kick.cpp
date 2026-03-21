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
  setParameter("enable_pivot_turn", true);
  setParameter("pivot_dribble_power", 0.6);
  setParameter("pivot_omega_limit", 3.0);
  setParameter("pivot_max_velocity", 1.0);
  setParameter("pivot_ball_lost_distance", 0.20);
  setParameter("pivot_exit_angle_deg", 5.0f);
  setParameter("pivot_turn_timeout", 3.0);

  addStateFunction(static_cast<int>(KickState::ENTRY_POINT), [this]() {
    visualizer->drawDebugLabel(robot()->pose.pos, "Kick::ENTRY_POINT");
    return Status::RUNNING;
  });

  addTransitions(
    static_cast<int>(KickState::ENTRY_POINT),
    {
      {static_cast<int>(KickState::PIVOT_CAPTURE),
       [this]() {
         if (!getParameter<bool>("enable_pivot_turn")) return false;
         return (world_model()->ball().pos - robot()->pose.pos).norm() < 0.20;
       }},
      {static_cast<int>(KickState::AROUND_BALL_AND_KICK), [this]() { return true; }},
    });

  addStateFunction(static_cast<int>(KickState::AROUND_BALL_AND_KICK), [this]() {
    pivot_turn_entry_time_.reset();
    Point ball_pos = world_model()->ball().pos;
    // 味方ペナルティエリア内のボールには接近しない（GKに委ねる）
    if (world_model()->point_checker.isFriendPenaltyArea(ball_pos, 0.15)) {
      command->lookAtBall();
      command->addPlanningFactor("kick", "WAIT_BALL_EXIT_FRIEND_PA");
      return Status::RUNNING;
    }
    const double interval = std::max(getParameter<double>("around_interval"), 0.01);
    const bool go_around_ball = getParameter<bool>("go_around_ball");
    const Vector2 kick_vec = computeKickVec();
    Point safe_target = ball_pos + kick_vec;

    // 視認性の高いキック方向の可視化: 太い矢印 + 角度しきい値の扇
    drawKickDirectionVisualization(ball_pos, kick_vec);
    visualizer->drawDebugLabel(robot()->pose.pos, "Kick::AROUND_BALL");
    Point approach = [&]() -> Point {
      if (!go_around_ball) {
        return ball_pos - kick_vec * interval;
      }
      // 改良回り込み: ロボット->目標線分に対するボール最近傍方向へ回り込み
      double max_interval = interval * 2.0;
      return computeAroundBallApproachTargetDynamic(
        ball_pos, safe_target, robot()->pose.pos, interval, max_interval);
    }();

    command->disableBallAvoidance();
    using boost::math::constants::degree;
    const double angle_threshold = getParameter<double>("angle_threshold_deg") * degree<double>();
    const bool angle_ok =
      std::abs(getAngleDiff(getAngle(kick_vec), getAngle(ball_pos - robot()->pose.pos))) <
      angle_threshold;

    double kick_vec_gain = [&]() {
      Segment ball_kick_zone{ball_pos, ball_pos - kick_vec * interval};
      // 角度が合っている場合のみキックゾーンに入ったと判断し、ボールに向かって押し込む
      if (angle_ok && bg::distance(ball_kick_zone, robot()->pose.pos) < 0.15) {
        command->disableCollisionAvoidance();
        return 0.5;
      } else {
        return 0.0;
      }
    }();

    command->lookAtFrom(safe_target, ball_pos)
      .setDribblerTargetPosition(approach + kick_vec * kick_vec_gain);
    if (angle_ok) {
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
      // ボール近傍では予防的にドリブラーを弱く起動し、接触時の弾きを防止
      double ball_dist = (ball_pos - robot()->pose.pos).norm();
      if (ball_dist < 0.3) {
        command->withDribble(0.3);
      } else {
        command->withDribble(0.0);
      }
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

  addStateFunction(static_cast<int>(KickState::PIVOT_CAPTURE), [this]() {
    pivot_turn_entry_time_.reset();
    Point ball_pos = world_model()->ball().pos;
    const Vector2 kick_vec = computeKickVec();
    Point safe_target = ball_pos + kick_vec;

    visualizer->drawDebugLabel(robot()->pose.pos, "Kick::PIVOT_CAPTURE");

    // ドリブラーをボールに当てながら、最終的なキック方向を向く
    command->disableBallAvoidance();
    command->disableCollisionAvoidance();
    command->setMaxVelocity("Kick::PIVOT_CAPTURE", getParameter<double>("pivot_max_velocity"));
    command->lookAtFrom(safe_target, ball_pos).setDribblerTargetPosition(ball_pos);
    command->withDribble(getParameter<double>("pivot_dribble_power"));
    command->kickStraight(0.0);

    return Status::RUNNING;
  });

  // ボールセンサー反応 → PIVOT_TURNへ
  addTransition(
    static_cast<int>(KickState::PIVOT_CAPTURE), static_cast<int>(KickState::PIVOT_TURN),
    [this]() { return robot()->ball_contact.findPastContact(0.05); });

  // ボールが遠ざかった（捕捉失敗）→ AROUND_BALL_AND_KICKにフォールバック
  addTransition(
    static_cast<int>(KickState::PIVOT_CAPTURE), static_cast<int>(KickState::AROUND_BALL_AND_KICK),
    [this]() {
      return (world_model()->ball().pos - robot()->pose.pos).norm() >
               getParameter<double>("pivot_ball_lost_distance") &&
             !world_model()->ball().isMoving(getParameter<double>("kicked_speed_threshold"));
    });

  addStateFunction(static_cast<int>(KickState::PIVOT_TURN), [this]() {
    if (!pivot_turn_entry_time_) {
      pivot_turn_entry_time_ = std::chrono::steady_clock::now();
    }
    Point ball_pos = world_model()->ball().pos;
    const Vector2 kick_vec = computeKickVec();
    Point safe_target = ball_pos + kick_vec;

    drawKickDirectionVisualization(ball_pos, kick_vec);
    visualizer->drawDebugLabel(robot()->pose.pos, "Kick::PIVOT_TURN");

    command->disableBallAvoidance();
    command->disableCollisionAvoidance();
    command->setOmegaLimit(getParameter<double>("pivot_omega_limit"));
    command->setMaxVelocity("Kick::PIVOT_TURN", getParameter<double>("pivot_max_velocity"));

    // lookAtFromが先（target_thetaを設定）、setDribblerTargetPositionが後（theta依存）
    command->lookAtFrom(safe_target, ball_pos).setDribblerTargetPosition(ball_pos);
    command->withDribble(getParameter<double>("pivot_dribble_power"));
    command->kickStraight(0.0);

    return Status::RUNNING;
  });

  // 角度が揃ったら AROUND_BALL_AND_KICK へ渡してキックを担当させる
  // ボールセンサー反応状態（PIVOT_CAPTURE確認済み）ではボールはキッカー前にあるため、
  // 幾何学的角度より robot()->pose.theta を使う方が位置誤差の影響を受けにくい
  addTransition(
    static_cast<int>(KickState::PIVOT_TURN), static_cast<int>(KickState::AROUND_BALL_AND_KICK),
    [this]() {
      const Vector2 kick_vec = computeKickVec();
      using boost::math::constants::degree;
      const double threshold = getParameter<double>("pivot_exit_angle_deg") * degree<double>();
      return std::abs(getAngleDiff(getAngle(kick_vec), robot()->pose.theta)) < threshold;
    });

  // ボール保持失敗（ボールが低速で離脱）→ AROUND_BALL_AND_KICKにフォールバック
  addTransition(
    static_cast<int>(KickState::PIVOT_TURN), static_cast<int>(KickState::AROUND_BALL_AND_KICK),
    [this]() {
      double ball_dist = (world_model()->ball().pos - robot()->kicker_center()).norm();
      return ball_dist > getParameter<double>("pivot_ball_lost_distance") &&
             !world_model()->ball().isMoving(getParameter<double>("kicked_speed_threshold"));
    });

  // タイムアウト → AROUND_BALL_AND_KICKにフォールバック（デッドロック防止）
  addTransition(
    static_cast<int>(KickState::PIVOT_TURN), static_cast<int>(KickState::AROUND_BALL_AND_KICK),
    [this]() {
      if (!pivot_turn_entry_time_) return false;
      auto elapsed = std::chrono::steady_clock::now() - *pivot_turn_entry_time_;
      return std::chrono::duration<double>(elapsed).count() >
             getParameter<double>("pivot_turn_timeout");
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

Vector2 Kick::computeKickVec() const
{
  auto target = getParameter<Point>("target");
  Point ball_pos = world_model()->ball().pos;
  Vector2 v = target - ball_pos;
  if (v.norm() > 1e-6) return v.normalized();
  v = robot()->pose.pos - ball_pos;
  if (v.norm() > 1e-6) return v.normalized();
  return Vector2(1.0, 0.0);
}

void Kick::drawKickDirectionVisualization(const Point & ball_pos, const Vector2 & kick_vec)
{
  double arrow_len = world_model()->fieldSize().x() * 0.5 + 0.5;
  visualizer->arrow(ball_pos, kick_vec, arrow_len, "lime", 20, 0.35, 0.20);
  using boost::math::constants::degree;
  double half_angle = getParameter<double>("angle_threshold_deg") * degree<double>();
  double base_theta = getAngle(kick_vec);
  constexpr double arc_radius = 0.9;
  auto dir_left = Vector2(std::cos(base_theta + half_angle), std::sin(base_theta + half_angle));
  auto dir_right = Vector2(std::cos(base_theta - half_angle), std::sin(base_theta - half_angle));
  visualizer->drawLine(ball_pos, ball_pos + dir_left * arc_radius, "white", 10, 0.6);
  visualizer->drawLine(ball_pos, ball_pos + dir_right * arc_radius, "white", 10, 0.6);
  visualizer->arc(
    ball_pos, arc_radius, base_theta - half_angle, base_theta + half_angle, "white", 10, 16);
}
}  // namespace crane::skills
