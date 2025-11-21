// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_geometry/geometry_operations.hpp>
#include <crane_robot_skills/kick.hpp>

#include "../include/crane_robot_skills/single_ball_placement.hpp"

namespace crane::skills
{

void Kick::initialize()
{
  command->usePositionMode();
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
  setParameter("moving_speed_threshold", 0.2);
  setParameter("kicked_speed_threshold", 1.5);

  receive_skill.setParameter("dribble_power", 0.3);
  receive_skill.setParameter("enable_software_bumper", false);
  receive_skill.setParameter("policy", std::string("min_slack"));
  receive_skill.setParameter("enable_active_receive", true);
  receive_skill.setParameter("enable_redirect", true);
  receive_skill.setParameter("redirect_target", Point(0, 0));
  receive_skill.setParameter("redirect_kick_power", 0.3);

  addStateFunction(KickState::ENTRY_POINT, [this]() {
    visualizer->drawDebugLabel(robot()->pose.pos, "Kick::ENTRY_POINT");
    return Status::RUNNING;
  });

  addTransition(KickState::ENTRY_POINT, KickState::AROUND_BALL_AND_KICK, [this]() { return true; });

  addStateFunction(KickState::POSITIVE_REDIRECT_KICK, [this]() {
    visualizer->drawDebugLabel(robot()->pose.pos, "Kick::POSITIVE_REDIRECT_KICK");
    // ボールラインに沿って追いかけつつ、角度はtargetへ向ける
    const auto & ball_pos = world_model()->ball().pos;
    command->lookAtFrom(getParameter<Point>("target"), ball_pos);

    const auto & ball_vel_normed = world_model()->ball().vel.normalized();
    Segment ball_line = world_model()->ball().getTrajectorySegmentByDistance(10.);
    auto [distance, closest_point] = getClosestPointAndDistance(ball_pos, ball_line);
    if ((ball_pos - closest_point).dot(ball_vel_normed) > 0) {
      // 通り過ぎていれば追いかけて蹴る
      auto target_pos = [&]() -> Point {
        if (distance < 0.1) {
          return ball_pos + ball_vel_normed;
        } else {
          return closest_point + ball_vel_normed * distance;
        }
      }();
      command->setDribblerTargetPosition(target_pos);
      // TODO(HansRobo): 速度指定対応
      command->kickStraight(0.3);
      command->disableBallAvoidance();
    } else {
      // まだだったら避ける
      command->setTargetPosition(
        closest_point + (robot()->pose.pos - closest_point).normalized() * 0.3);
    }

    return Status::RUNNING;
  });

  addTransition(KickState::POSITIVE_REDIRECT_KICK, KickState::ENTRY_POINT, [this]() {
    return !world_model()->ball().isMovingAwayFrom(robot()->pose.pos, 10.0) or
           !world_model()->ball().isMovingTowards(getParameter<Point>("target"), 30.0);
  });

  addStateFunction(KickState::REDIRECT_KICK, [this]() {
    visualizer->drawDebugLabel(robot()->pose.pos, "Kick::REDIRECT_KICK");
    receive_skill.setParameter("target", getParameter<Point>("target"));
    if (robot()->getDistance(world_model()->ball().pos) < 0.5) {
      receive_skill.setParameter("policy", std::string("closest"));
    } else {
      receive_skill.setParameter("policy", std::string("min_slack"));
    }
    command->disableBallAvoidance();
    return receive_skill.update();
  });

  addTransition(KickState::REDIRECT_KICK, KickState::AROUND_BALL_AND_KICK, [this]() {
    // ボールが止まったら回り込みへ
    return not world_model()->ball().isMoving(getParameter<double>("moving_speed_threshold"));
  });

  addTransition(KickState::REDIRECT_KICK, KickState::ENTRY_POINT, [this]() {
    // 素早く遠ざかっていったら終了
    return world_model()->ball().isMoving(getParameter<double>("kicked_speed_threshold")) &&
           world_model()->ball().isMovingAwayFrom(robot()->pose.pos, 30.);
  });

  addStateFunction(KickState::AROUND_BALL_AND_KICK, [this]() {
    auto target = getParameter<Point>("target");
    Point ball_pos = world_model()->ball().pos;
    // 視認性の高いキック方向の可視化: 太い矢印 + 角度しきい値の扇
    {
      Vector2 dir = (target - ball_pos).normalized();
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
    // 改良回り込み: 固定中間点ではなく、ロボット→基準点の線分に対するボール最近傍方向へ回り込み
    constexpr double INTERVAL = 0.15;
    constexpr double MAX_INTERVAL = 0.3;  // 大回り上限
    Point approach = computeAroundBallApproachTargetDynamic(
      ball_pos, target, robot()->pose.pos, INTERVAL, MAX_INTERVAL);

    Vector2 kick_vec = (target - ball_pos).normalized();
    double kick_vec_gain = [&]() {
      Segment ball_kick_zone{ball_pos, ball_pos - kick_vec * INTERVAL};
      if (bg::distance(ball_kick_zone, robot()->pose.pos) < 0.1) {
        command->disableCollisionAvoidance();
        return 0.5;
      } else {
        return 0.0;
      }
    }();

    command->setTargetPosition(approach + kick_vec * kick_vec_gain).lookAtFrom(target, ball_pos);
    command->disableBallAvoidance();
    using boost::math::constants::degree;
    if (
      std::abs(getAngleDiff(getAngle(target - ball_pos), getAngle(ball_pos - robot()->pose.pos))) <
      20. * degree<double>()) {
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

  addTransition(KickState::AROUND_BALL_AND_KICK, KickState::ENTRY_POINT, [this]() {
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
  using boost::math::constants::degree;
  // if (
  //   getAngleDiff(
  //     getAngle(getParameter<Point>("target") - world_model()->ball().pos), robot()->pose.theta) <
  //   getParameter<double>("angle_threshold_deg") * degree<double>()) {
  if (getParameter<bool>("use_target_chip_distance")) {
    command->setKickWithChipTargetDistance(getParameter<double>("target_chip_distance"));
  } else {
    command->kickWithChip(getParameter<double>("kick_power"));
  }
  // } else {
  //   command->kickStraight(0.0);
  // }
}

auto Kick::kickStraight() -> void
{
  using boost::math::constants::degree;
  // if (
  //   getAngleDiff(
  //     getAngle(getParameter<Point>("target") - world_model()->ball().pos), robot()->pose.theta) <
  //   getParameter<double>("angle_threshold_deg") * degree<double>()) {
  if (getParameter<bool>("use_target_kick_speed")) {
    command->setKickStraightTargetSpeed(getParameter<double>("target_kick_speed"));
  } else {
    command->kickStraight(getParameter<double>("kick_power"));
  }
  // } else {
  //   command->kickStraight(0.0);
  // }
}
}  // namespace crane::skills
