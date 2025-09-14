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
  setParameter("moving_speed_threshold", 0.2);
  setParameter("kicked_speed_threshold", 1.5);
  // CHASE_BALL_AND_KICK 用パラメータ
  setParameter("chase_dribble_power", 0.5);
  setParameter("capture_contact_time", 0.1);  // 捕捉判定のための接触時間 [s]
  setParameter("decel_target_speed", 0.3);    // この速度まで減速する目標値 [m/s]
  setParameter("turn_offset", 0.15);          // 回転時のボール背後オフセット [m]
  setParameter("turn_dribble_power", 0.4);
  setParameter("turn_omega_limit", 10.0);        // 回転時の角速度上限 [rad/s]
  setParameter("chase_enable_lateral", 0.5);     // 追従開始の横ずれ許容距離 [m]
  setParameter("chase_enable_ahead_dist", 2.0);  // ボール先方の最近傍点までの許容距離 [m]

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

  // ボールの移動と自ロボット位置関係に応じて CHASE または AROUND を選択
  addTransition(KickState::ENTRY_POINT, KickState::CHASE_BALL_AND_KICK, [this]() {
    // 条件: ボールが十分動いている かつ ロボットが追従に適した位置にいる
    const auto & ball = world_model()->ball();
    if (!ball.isMoving(getParameter<double>("moving_speed_threshold"))) {
      return false;
    }
    // ボール軌道に対する自位置の最近傍点を求める
    Segment ball_line = ball.getTrajectorySegmentByDistance(10.0);
    auto res = ball.getClosestPointToTrajectory(robot()->pose.pos, 10.0);
    Point closest = res.closest_point;
    // 最近傍点がボールの前方（進行方向側）にあるか
    bool ahead = (closest - ball.pos).dot(ball.vel) > 0.0;
    // 自位置から軌道への横ずれと、ボールから最近傍点までの「先行距離」を評価
    auto gd = getClosestPointAndDistance(robot()->pose.pos, ball_line);
    double lateral = gd.distance;
    double ahead_dist = (closest - ball.pos).norm();
    bool lateral_ok = lateral < getParameter<double>("chase_enable_lateral");
    bool ahead_ok = ahead && (ahead_dist < getParameter<double>("chase_enable_ahead_dist"));
    return lateral_ok && ahead_ok;
  });
  addTransition(KickState::ENTRY_POINT, KickState::AROUND_BALL_AND_KICK, [this]() {
    // CHASE 条件を満たさない場合は回り込みへ（ボールが止まっている or 追従に不向き）
    const auto & ball = world_model()->ball();
    if (!ball.isMoving(getParameter<double>("moving_speed_threshold"))) {
      return true;
    }
    Segment ball_line = ball.getTrajectorySegmentByDistance(10.0);
    auto res = ball.getClosestPointToTrajectory(robot()->pose.pos, 10.0);
    Point closest = res.closest_point;
    bool ahead = (closest - ball.pos).dot(ball.vel) > 0.0;
    auto gd = getClosestPointAndDistance(robot()->pose.pos, ball_line);
    double lateral = gd.distance;
    double ahead_dist = (closest - ball.pos).norm();
    bool lateral_ok = lateral < getParameter<double>("chase_enable_lateral");
    bool ahead_ok = ahead && (ahead_dist < getParameter<double>("chase_enable_ahead_dist"));
    return !(lateral_ok && ahead_ok);
  });

  addStateFunction(KickState::POSITIVE_REDIRECT_KICK, [this]() {
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text("Kick::POSITIVE_REDIRECT_KICK")
      .fill("white")
      .fontSize(100)
      .build();
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
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text("Kick::REDIRECT_KICK")
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
    visualizer->line()
      .start(ball_pos)
      .end(ball_pos + (target - ball_pos).normalized() * 1.0)
      .stroke("blue")
      .strokeWidth(10)
      .build();
    {
      visualizer->text()
        .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
        .text("Kick::AROUND_BALL")
        .fill("white")
        .fontSize(100)
        .build();
    }
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

  // --- CHASE_BALL_AND_KICK（追従・捕捉・回転整列） ---
  addStateFunction(KickState::CHASE_BALL_AND_KICK, [this]() {
    auto target = getParameter<Point>("target");
    auto & ball = world_model()->ball();
    const double moving_thr = getParameter<double>("moving_speed_threshold");
    const double capture_contact_time = getParameter<double>("capture_contact_time");
    const double decel_target = getParameter<double>("decel_target_speed");
    const double turn_offset = getParameter<double>("turn_offset");

    // 状態表示
    visualizer->text()
      .position(robot()->pose.pos.x() - 0.5, robot()->pose.pos.y() + 0.5)
      .text("Kick::CHASE_BALL")
      .fill("white")
      .fontSize(100)
      .build();

    // 条件に応じてサブ挙動を分岐
    bool has_contact = robot()->ball_contact.findPastContact(capture_contact_time);
    bool angle_aligned = [&]() {
      using boost::math::constants::degree;
      return std::abs(
               getAngleDiff(getAngle(target - ball.pos), getAngle(ball.pos - robot()->pose.pos))) <
             getParameter<double>("angle_threshold_deg") * degree<double>();
    }();

    if (!has_contact) {
      // 1) 追従・捕捉: 転がるボールに追いつき、近未来軌道の最近傍点で捉える
      Segment ball_line = ball.getTrajectorySegmentByDistance(10.0);
      auto result = ball.getClosestPointToTrajectory(robot()->pose.pos, 10.0);
      Point intercept = result.closest_point;
      // インターセプト点にドリブラ先導し、ボールへ向ける
      command->setDribblerTargetPosition(intercept)
        .lookAtBallFrom(intercept)
        .dribble(getParameter<double>("chase_dribble_power"))
        .disableBallAvoidance();

      // 安全策: 行き過ぎ防止のため、距離に応じて終端速度を制限
      double dist = robot()->getDistance(intercept);
      command->setMaxVelocity("Kick::CHASE_APPROACH", std::clamp(dist, 0.5, 2.5));
      command->kickStraight(0.0);
    } else if (ball.vel.norm() > decel_target + 1e-3) {
      // 2) 減速: ボールを保持したまま目標速度まで減速
      command->dribble(std::max(0.5, getParameter<double>("chase_dribble_power")))
        .lookAtBall()
        .setMaxVelocity("Kick::CHASE_DECEL", decel_target)
        .kickStraight(0.0)
        .disableBallAvoidance();
    } else {
      // 3) 回転整列: 一定オフセットの回り込み点を目標に、ドリブルしながらターゲットへ向けて回転
      Point approach = computeAroundBallApproachTargetDynamic(
        ball.pos, target, robot()->pose.pos, turn_offset, turn_offset);
      command->setTargetPosition(approach)
        .lookAtFrom(target, ball.pos)
        .withDribble(getParameter<double>("turn_dribble_power"))
        .setOmegaLimit(getParameter<double>("turn_omega_limit"))
        .disableBallAvoidance();

      if (angle_aligned) {
        if (getParameter<bool>("chip_kick")) {
          kickWithChip();
        } else {
          kickStraight();
        }
      } else {
        command->kickStraight(0.0);
      }
    }

    return Status::RUNNING;
  });

  addTransition(KickState::CHASE_BALL_AND_KICK, KickState::AROUND_BALL_AND_KICK, [this]() {
    // ボールが十分に動いていない場合は、回り込み挙動へ切り替え
    return !world_model()->ball().isMoving(getParameter<double>("moving_speed_threshold"));
  });

  addTransition(KickState::CHASE_BALL_AND_KICK, KickState::ENTRY_POINT, [this]() {
    // キック後にボールが離れていったら終了
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
