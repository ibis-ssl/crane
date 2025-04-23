// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/kick.hpp>

namespace crane::skills
{

void Kick::initialize()
{
  command->usePositionMode();
  setParameter("target", Point(0, 0));
  setParameter("kick_power", 0.7f);
  setParameter("chip_kick", false);
  setParameter("with_dribble", false);
  setParameter("dribble_power", 0.3f);
  setParameter("angle_threshold", 0.1f);
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
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text("Kick::ENTRY_POINT")
      .fill("white")
      .fontSize(100)
      .build();
    return Status::RUNNING;
  });

  addTransition(KickState::ENTRY_POINT, KickState::AROUND_BALL_AND_KICK, [this]() { return true; });

  addStateFunction(KickState::POSITIVE_REDIRECT_KICK, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text("Kick::POSITIVE_REDIRECT_KICK")
      .fill("white")
      .fontSize(100)
      .build();
    // ボールラインに沿って追いかけつつ、角度はtargetへ向ける
    const auto & ball_pos = world_model()->ball.pos;
    command->lookAtFrom(getParameter<Point>("target"), ball_pos);

    const auto & ball_vel_normed = world_model()->ball.vel.normalized();
    Segment ball_line{ball_pos - ball_vel_normed * 10, ball_pos + ball_vel_normed * 10};
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
    return !world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 10.0) or
           !world_model()->ball.isMovingTowards(getParameter<Point>("target"), 30.0);
  });

  addStateFunction(KickState::REDIRECT_KICK, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text("Kick::REDIRECT_KICK")
      .fill("white")
      .fontSize(100)
      .build();
    receive_skill.setParameter("target", getParameter<Point>("target"));
    if (robot()->getDistance(world_model()->ball.pos) < 0.5) {
      receive_skill.setParameter("policy", std::string("closest"));
    } else {
      receive_skill.setParameter("policy", std::string("min_slack"));
    }
    command->disableBallAvoidance();
    return receive_skill.update();
  });

  addTransition(KickState::REDIRECT_KICK, KickState::AROUND_BALL_AND_KICK, [this]() {
    // ボールが止まったら回り込みへ
    return not world_model()->ball.isMoving(getParameter<double>("moving_speed_threshold"));
  });

  addTransition(KickState::REDIRECT_KICK, KickState::ENTRY_POINT, [this]() {
    // 素早く遠ざかっていったら終了
    return world_model()->ball.isMoving(getParameter<double>("kicked_speed_threshold")) &&
           world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 30.);
  });

  addStateFunction(KickState::AROUND_BALL_AND_KICK, [this]() {
    auto target = getParameter<Point>("target");
    Point ball_pos = world_model()->ball.pos;
    visualizer->line()
      .start(ball_pos)
      .end(ball_pos + (target - ball_pos).normalized() * 1.0)
      .stroke("blue")
      .strokeWidth(10)
      .build();
    constexpr double SWITCH_DISTANCE = 1.0;
    {
      visualizer->circle()
        .center(ball_pos)
        .radius(SWITCH_DISTANCE)
        .stroke("yellow")
        .strokeWidth(10)
        .build();
    }
    if (robot()->getDistance(ball_pos) > SWITCH_DISTANCE) {
      {
        visualizer->text()
          .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
          .text("Kick::AROUND_BALL(遠い)")
          .fill("white")
          .fontSize(100)
          .build();
      }
      command->setTargetPosition(ball_pos + (ball_pos - target).normalized() * 0.3)
        .lookAtFrom(target, ball_pos)
        .setTerminalVelocity(0.4);
      return Status::RUNNING;
    } else {
      {
        visualizer->text()
          .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
          .text("Kick::AROUND_BALL（近い）")
          .fill("white")
          .fontSize(100)
          .build();
      }
      auto calculateRatio =
        [](const double distance, const double min_distance, const double max_distance) {
          return (distance - min_distance) / (max_distance - min_distance);
        };

      // ボールを避けて回り込む
      using boost::math::constants::degree;
      double ratio =
        1.5 + std::clamp(
                -calculateRatio(robot()->getDistance(world_model()->ball.pos), 0.2, 1.5), -0.5, 0.);

      double move_direction = getAngle(target - robot()->pose.pos) +
                              (getAngleDiff(
                                getAngle(world_model()->ball.pos - robot()->pose.pos),
                                getAngle(target - robot()->pose.pos))) *
                                ratio;
      Vector2 move_vec = getNormVec(move_direction);
      double move_vec_gain = [&]() {
        if (
          getAngleDiff(getAngle(target - ball_pos), robot()->pose.theta) < 10. * degree<double>()) {
          return 0.4;
        } else {
          return 0.2;
        }
      }();

      command->lookAtFrom(target, ball_pos)
        .setDribblerTargetPosition(
          robot()->pose.pos + move_vec * move_vec_gain + world_model()->ball.vel * 0.3)
        // .setTerminalVelocity(world_model()->ball.vel.norm())
        .disableCollisionAvoidance()
        .disableBallAvoidance();

      if (
        std::abs(
          getAngleDiff(getAngle(target - ball_pos), getAngle(ball_pos - robot()->pose.pos))) <
        20. * degree<double>()) {
        if (getParameter<bool>("chip_kick")) {
          command->kickWithChip(getParameter<double>("kick_power"));
        } else {
          command->kickStraight(getParameter<double>("kick_power"));
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
    }
  });

  addTransition(KickState::AROUND_BALL_AND_KICK, KickState::ENTRY_POINT, [this]() {
    // 素早く遠ざかっていったら終了
    return world_model()->ball.isMoving(getParameter<double>("kicked_speed_threshold")) &&
           world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 30.);
  });
}

auto Kick::getBallExitPointFromField(const double offset) -> Point
{
  Segment ball_line{
    world_model()->ball.pos, world_model()->ball.pos + world_model()->ball.vel.normalized() * 10.0};

  const double X = world_model()->field_size.x() / 2.0 - offset;
  const double Y = world_model()->field_size.y() / 2.0 - offset;

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
  return world_model()->ball.pos;
}
}  // namespace crane::skills
