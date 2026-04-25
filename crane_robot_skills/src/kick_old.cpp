// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/kick_old.hpp>
#include <magic_enum/magic_enum.hpp>

#include "../include/crane_robot_skills/single_ball_placement.hpp"

namespace crane::skills
{
std::string KickOld::getStateName(int s)
{
  return std::string(magic_enum::enum_name(static_cast<KickOldState>(s)));
}

void KickOld::initialize()
{
  using S = KickOldState;
  auto s = [](S state) { return static_cast<int>(state); };

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

  addStateFunction(s(S::ENTRY_POINT), [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text("KickOld::ENTRY_POINT")
      .fill("white")
      .fontSize(100)
      .build();
    return Status::RUNNING;
  });

  addTransition(s(S::ENTRY_POINT), s(S::AROUND_BALL_AND_KICK), [this]() { return true; });

  addStateFunction(s(S::POSITIVE_REDIRECT_KICK), [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text("KickOld::POSITIVE_REDIRECT_KICK")
      .fill("white")
      .fontSize(100)
      .build();
    const auto & ball_pos = world_model()->ball().pos;
    command->lookAtFrom(getParameter<Point>("target"), ball_pos);

    const auto & ball_vel_normed = world_model()->ball().vel.normalized();
    Segment ball_line = world_model()->ball().getTrajectorySegmentByDistance(10.);
    auto [distance, closest_point] = getClosestPointAndDistance(ball_pos, ball_line);
    if ((ball_pos - closest_point).dot(ball_vel_normed) > 0) {
      auto target_pos = [&]() -> Point {
        if (distance < 0.1) {
          return ball_pos + ball_vel_normed;
        } else {
          return closest_point + ball_vel_normed * distance;
        }
      }();
      command->setDribblerTargetPosition(target_pos);
      command->kickStraight(0.3);
      command->disableBallAvoidance();
    } else {
      command->setTargetPosition(
        closest_point + (robot()->pose.pos - closest_point).normalized() * 0.3);
    }

    return Status::RUNNING;
  });

  addTransition(s(S::POSITIVE_REDIRECT_KICK), s(S::ENTRY_POINT), [this]() {
    return !world_model()->ball().isMovingAwayFrom(robot()->pose.pos, 10.0) ||
           !world_model()->ball().isMovingTowards(getParameter<Point>("target"), 30.0);
  });

  addStateFunction(s(S::REDIRECT_KICK), [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text("KickOld::REDIRECT_KICK")
      .fill("white")
      .fontSize(100)
      .build();
    receive_skill.setParameter("target", getParameter<Point>("target"));
    if (robot()->getDistance(world_model()->ball().pos) < 0.5) {
      receive_skill.setParameter("policy", std::string("closest"));
    } else {
      receive_skill.setParameter("policy", std::string("min_slack"));
    }
    command->disableBallAvoidance();
    return receive_skill.update();
  });

  addTransition(s(S::REDIRECT_KICK), s(S::AROUND_BALL_AND_KICK), [this]() {
    return !world_model()->ball().isMoving(getParameter<double>("moving_speed_threshold"));
  });

  addTransition(s(S::REDIRECT_KICK), s(S::ENTRY_POINT), [this]() {
    return world_model()->ball().isMoving(getParameter<double>("kicked_speed_threshold")) &&
           world_model()->ball().isMovingAwayFrom(robot()->pose.pos, 30.);
  });

  addStateFunction(s(S::AROUND_BALL_AND_KICK), [this]() {
    auto target = getParameter<Point>("target");
    Point ball_pos = world_model()->ball().pos;
    visualizer->line()
      .start(ball_pos)
      .end(ball_pos + (target - ball_pos).normalized() * 1.0)
      .stroke("blue")
      .strokeWidth(10)
      .build();

    constexpr double SWITCH_DISTANCE = 0.5;
    visualizer->circle()
      .center(ball_pos)
      .radius(SWITCH_DISTANCE)
      .stroke("yellow")
      .strokeWidth(10)
      .build();

    if (robot()->getDistance(ball_pos) > SWITCH_DISTANCE) {
      visualizer->text()
        .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
        .text("KickOld::AROUND_BALL(遠い)")
        .fill("white")
        .fontSize(100)
        .build();
      // 遠距離フェーズ: ボール後方の固定点へ向かいつつ角度を合わせる
      command->setTargetPosition(ball_pos + (ball_pos - target).normalized() * 0.3)
        .lookAtFrom(target, ball_pos);
      return Status::RUNNING;
    } else {
      visualizer->text()
        .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
        .text("KickOld::AROUND_BALL（近い）")
        .fill("white")
        .fontSize(100)
        .build();

      // 近距離フェーズ: ロボット相対の move_direction ベクトルで回り込む
      auto calculateRatio =
        [](const double distance, const double min_distance, const double max_distance) {
          return (distance - min_distance) / (max_distance - min_distance);
        };

      using boost::math::constants::degree;
      double ratio =
        1.5 +
        std::clamp(
          -calculateRatio(robot()->getDistance(world_model()->ball().pos), 0.2, 1.5), -0.5, 0.);

      double move_direction = getAngle(target - robot()->pose.pos) +
                              (getAngleDiff(
                                getAngle(world_model()->ball().pos - robot()->pose.pos),
                                getAngle(target - robot()->pose.pos))) *
                                ratio;
      Vector2 move_vec = getNormVec(move_direction);
      double move_vec_gain = [&]() {
        if (
          getAngleDiff(getAngle(target - ball_pos), robot()->pose.theta) < 2.5 * degree<double>()) {
          return 0.4;
        } else {
          return 0.2;
        }
      }();

      Vector2 ball_away_vec = (robot()->pose.pos - world_model()->ball().pos).normalized();
      double ball_away_gain = 0.0;
      if (
        robot()->getDistance(world_model()->ball().pos) < 0.2 &&
        getAngleDiff(
          getAngle(target - ball_pos), getAngle(world_model()->ball().pos - robot()->pose.pos)) >
          10. * degree<double>()) {
        ball_away_gain = 0.0;
      }

      command->lookAtFrom(target, ball_pos)
        .setDribblerTargetPosition(
          robot()->pose.pos + move_vec * move_vec_gain + world_model()->ball().vel * 0.5 +
          ball_away_vec * ball_away_gain)
        .disableCollisionAvoidance()
        .disableBallAvoidance();

      if (
        std::abs(
          getAngleDiff(getAngle(target - ball_pos), getAngle(ball_pos - robot()->pose.pos))) <
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
        command->withDribble(0.0);
      }
      return Status::RUNNING;
    }
  });

  addTransition(s(S::AROUND_BALL_AND_KICK), s(S::ENTRY_POINT), [this]() {
    return world_model()->ball().isMoving(getParameter<double>("kicked_speed_threshold")) &&
           world_model()->ball().isMovingAwayFrom(robot()->pose.pos, 30.);
  });
}

auto KickOld::getBallExitPointFromField(const double offset) -> Point
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

auto KickOld::kickWithChip() -> void
{
  if (getParameter<bool>("use_target_chip_distance")) {
    command->setKickWithChipTargetDistance(getParameter<double>("target_chip_distance"));
  } else {
    command->kickWithChip(getParameter<double>("kick_power"));
  }
}

auto KickOld::kickStraight() -> void
{
  if (getParameter<bool>("use_target_kick_speed")) {
    command->setKickStraightTargetSpeed(getParameter<double>("target_kick_speed"));
  } else {
    command->kickStraight(getParameter<double>("kick_power"));
  }
}
}  // namespace crane::skills
