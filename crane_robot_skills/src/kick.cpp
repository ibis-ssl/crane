// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/kick.hpp>

namespace crane::skills
{

Kick::Kick(RobotCommandWrapperBase::SharedPtr & base)
: SkillBaseWithState<KickState>("Kick", base, KickState::ENTRY_POINT),
  receive_skill(std::make_shared<Receive>(base)),
  phase(getContextReference<std::string>("phase"))
{
  setParameter("target", Point(0, 0));
  setParameter("kick_power", 0.5f);
  setParameter("chip_kick", false);
  setParameter("with_dribble", false);
  setParameter("dribble_power", 0.3f);
  setParameter("dot_threshold", 0.95f);
  setParameter("angle_threshold", 0.1f);
  setParameter("around_interval", 0.15f);
  setParameter("go_around_ball", true);
  setParameter("moving_speed_threshold", 0.2);
  setParameter("kicked_speed_threshold", 1.5);

  receive_skill->setParameter("dribble_power", 0.3);
  receive_skill->setParameter("enable_software_bumper", false);
  receive_skill->setParameter("policy", std::string("min_slack"));
  receive_skill->setParameter("enable_active_receive", true);
  receive_skill->setParameter("enable_redirect", true);
  receive_skill->setParameter("redirect_target", Point(0, 0));
  receive_skill->setParameter("redirect_kick_power", 0.3);

  addStateFunction(KickState::ENTRY_POINT, [this]() {
    visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::ENTRY_POINT");
    return Status::RUNNING;
  });

  addTransition(KickState::ENTRY_POINT, KickState::CHASE_BALL, [this]() {
    return world_model()->ball.isMoving(getParameter<double>("moving_speed_threshold"));
  });

  addTransition(KickState::ENTRY_POINT, KickState::AROUND_BALL, [this]() { return true; });

  addStateFunction(KickState::CHASE_BALL, [this]() {
    std::stringstream state;
    state << "Kick::CHASE_BALL::";
    // メモ：ボールが近い時はボールから少しずらした位置を目指したほうがいいかも
    auto [min_slack_pos, max_slack_pos] =
      world_model()->getMinMaxSlackInterceptPoint({robot()}, 5.0, 0.1, -0.3, 1., 2.0);
    if (min_slack_pos) {
      state << "min_slack: " << min_slack_pos.value().x() << ", " << min_slack_pos.value().y();
      command.setTargetPosition(min_slack_pos.value()).lookAtBallFrom(min_slack_pos.value());
    } else {
      // ball_lineとフィールドラインの交点を目指す
      Point ball_exit_point = getBallExitPointFromField(0.3);
      command.setTargetPosition(ball_exit_point)
        .lookAtFrom(world_model()->ball.pos, ball_exit_point);
      state << "ball_exit: " << ball_exit_point.x() << ", " << ball_exit_point.y();
    }
    visualizer->addPoint(robot()->pose.pos, 0, "", 1., state.str());
    return Status::RUNNING;
  });

  addTransition(KickState::CHASE_BALL, KickState::AROUND_BALL, [this]() {
    // ボールが止まったら回り込みへ
    command.disableBallAvoidance();
    return not world_model()->ball.isMoving(getParameter<double>("moving_speed_threshold"));
  });

  addTransition(KickState::CHASE_BALL, KickState::REDIRECT_KICK, [this]() {
    // ボールライン上に乗ったらリダイレクトキックへ
    command.disableBallAvoidance();
    return world_model()->ball.isMovingTowards(robot()->pose.pos, 10.0) &&
           getAngleDiff(
             getAngle(world_model()->ball.vel),
             getAngle(getParameter<Point>("target") - robot()->pose.pos) < M_PI / 2.0);
  });

  addTransition(KickState::CHASE_BALL, KickState::POSITIVE_REDIRECT_KICK, [this]() {
    command.disableBallAvoidance();
    return world_model()->ball.isMovingTowards(robot()->pose.pos, 10.0);
  });

  addStateFunction(KickState::POSITIVE_REDIRECT_KICK, [this]() {
    visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::POSITIVE_REDIRECT_KICK");
    // ボールラインに沿って追いかけつつ、角度はtargetへ向ける
    const auto & ball_pos = world_model()->ball.pos;
    command.lookAtFrom(getParameter<Point>("target"), ball_pos);

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
      command.setDribblerTargetPosition(target_pos);
      command.kickStraight(0.3);
      command.disableBallAvoidance();
    } else {
      // まだだったら避ける
      command.setTargetPosition(
        closest_point + (robot()->pose.pos - closest_point).normalized() * 0.3);
    }

    return Status::RUNNING;
  });

  addTransition(KickState::POSITIVE_REDIRECT_KICK, KickState::ENTRY_POINT, [this]() {
    return !world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 10.0) or
           !world_model()->ball.isMovingTowards(getParameter<Point>("target"), 30.0);
  });

  addStateFunction(KickState::REDIRECT_KICK, [this]() {
    visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::REDIRECT_KICK");
    receive_skill->setParameter("target", getParameter<Point>("target"));
    if (robot()->getDistance(world_model()->ball.pos) < 0.5) {
      receive_skill->setParameter("policy", std::string("closest"));
    } else {
      receive_skill->setParameter("policy", std::string("min_slack"));
    }
    command.disableBallAvoidance();
    return receive_skill->update();
  });

  addTransition(KickState::REDIRECT_KICK, KickState::AROUND_BALL, [this]() {
    // ボールが止まったら回り込みへ
    return not world_model()->ball.isMoving(getParameter<double>("moving_speed_threshold"));
  });

  addTransition(KickState::REDIRECT_KICK, KickState::ENTRY_POINT, [this]() {
    // 素早く遠ざかっていったら終了
    return world_model()->ball.isMoving(getParameter<double>("kicked_speed_threshold")) &&
           world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 30.);
  });

  addStateFunction(KickState::AROUND_BALL, [this]() {
    visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::AROUND_BALL");
    auto target = getParameter<Point>("target");
    Point ball_pos = world_model()->ball.pos;

    constexpr double SWITCH_DISTANCE = 0.5;
    if (robot()->getDistance(ball_pos) > SWITCH_DISTANCE) {
      command
        .setTargetPosition(
          ball_pos + (robot()->pose.pos - ball_pos).normalized() * (SWITCH_DISTANCE - 0.2))
        .lookAtFrom(target, ball_pos)
        .setTerminalVelocity(0.1);
      return Status::RUNNING;
    } else {
      auto calculateRatio =
        [](const double distance, const double min_distance, const double max_distance) {
          return (distance - min_distance) / (max_distance - min_distance);
        };

      // ボールを避けて回り込む
      using boost::math::constants::degree;
      double ratio =
        1.0 +
        std::clamp(
          0.5 - calculateRatio(robot()->getDistance(world_model()->ball.pos), 0., 2.0), 0., 0.5);

      double move_direction =
        robot()->pose.theta +
        (getAngle(world_model()->ball.pos - robot()->pose.pos) - robot()->pose.theta) * ratio;
      Vector2 move_vec = getNormVec(move_direction);
      command.setDribblerTargetPosition(robot()->pose.pos + move_vec * 0.1)
        .setTerminalVelocity(robot()->getDistance(world_model()->ball.pos) * 0.5 + 0.5)
        .lookAtFrom(target, ball_pos)
        .enableCollisionAvoidance()
        .disableBallAvoidance();

      if (getParameter<bool>("chip_kick")) {
        command.kickWithChip(getParameter<double>("kick_power"));
      } else {
        command.kickStraight(getParameter<double>("kick_power"));
      }
      if (getParameter<bool>("with_dribble")) {
        command.dribble(getParameter<double>("dribble_power"));
      } else {
        // ドリブラーを止める
        command.withDribble(0.0);
      }

      return Status::RUNNING;
    }
  });

  // 一旦KICKは使わない
  // addTransition(KickState::AROUND_BALL, KickState::KICK, [this]() {
  //   // 中間地点に到達したらキックへ
  //
  //   auto & ball_pos = world_model()->ball.pos;
  //   auto target = getParameter<Point>("target");
  //   Vector2 robot_to_ball = ball_pos - robot()->pose.pos;
  //   double dot = robot_to_ball.normalized().dot((target - ball_pos).normalized());
  //   using boost::math::constants::degree;
  //   return dot < std::cos(10. * degree<double>()) && robot()->getDistance(ball_pos) < 0.3;
  // });

  addStateFunction(KickState::KICK, [this]() {
    visualizer->addPoint(robot()->pose.pos, 0, "", 1., "Kick::KICK");
    auto target = getParameter<Point>("target");
    Point ball_pos = world_model()->ball.pos;
    command.setTargetPosition(ball_pos + (target - ball_pos).normalized() * 0.1)
      .setTerminalVelocity(0.5)
      .disableCollisionAvoidance()
      .disableBallAvoidance();
    if (getParameter<bool>("chip_kick")) {
      command.kickWithChip(getParameter<double>("kick_power"));
    } else {
      command.kickStraight(getParameter<double>("kick_power"));
    }
    if (getParameter<bool>("with_dribble")) {
      command.dribble(getParameter<double>("dribble_power"));
    } else {
      // ドリブラーを止める
      command.withDribble(0.0);
    }
    return Status::RUNNING;
  });

  addTransition(KickState::AROUND_BALL, KickState::ENTRY_POINT, [this]() {
    // 素早く遠ざかっていったら終了
    return world_model()->ball.isMoving(getParameter<double>("kicked_speed_threshold")) &&
           world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 30.);
  });

  addTransition(KickState::KICK, KickState::ENTRY_POINT, [this]() {
    // 素早く遠ざかっていったら終了
    return world_model()->ball.isMoving(getParameter<double>("kicked_speed_threshold")) &&
           world_model()->ball.isMovingAwayFrom(robot()->pose.pos, 30.);
  });

  addTransition(KickState::KICK, KickState::ENTRY_POINT, [this]() -> bool {
    // 素早く遠ざかっていったら終了
    auto target = getParameter<Point>("target");
    Point ball_pos = world_model()->ball.pos;
    Point p = ball_pos + (target - ball_pos).normalized() * 0.3;
    return robot()->getDistance(p) < 0.1;
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
