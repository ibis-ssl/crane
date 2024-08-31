// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/steal_ball_vel.hpp>
#include <ranges>

namespace crane::skills
{
StealBallVel::StealBallVel(RobotCommandWrapperBase::SharedPtr & base)
: SkillBaseWithState<StealBallVelState, RobotCommandWrapperSimpleVelocity>(
    "StealBallVel", base, StealBallVelState::MOVE_TO_FRONT)
{
  setParameter("突入速度", 1.0);
  setParameter("振り切り速度", 1.0);
  addStateFunction(
    StealBallVelState::MOVE_TO_FRONT,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      // ボールの正面に移動
      // 到着判定すると遅くなるので、敵ロボットにボールが隠されていなかったら次に行ってもいいかも
      auto theirs = world_model()->theirs.getAvailableRobots() | std::views::filter([](const auto& r) { return r != nullptr; });
      if (not theirs.empty()) {
        auto [ball_holder, distance] =
          world_model()->getNearestRobotWithDistanceFromPoint(world_model()->ball.pos, theirs);
        Point target_pos = world_model()->ball.pos + getNormVec(ball_holder->pose.theta) * 0.3;
        command.setTargetPosition(target_pos);
        command.lookAtBallFrom(target_pos);
        command.setMaxVelocity(1.0);
        if ((robot()->pose.pos - target_pos).norm() < 0.2) {
          skill_state = Status::SUCCESS;
        } else {
          skill_state = Status::RUNNING;
        }
      }
      return Status::RUNNING;
    });

  // 正面に移動したら突っ込んでボールを奪う
  addTransition(StealBallVelState::MOVE_TO_FRONT, StealBallVelState::STEAL, [this]() {
    return skill_state == Status::SUCCESS;
  });

  addTransition(StealBallVelState::STEAL, StealBallVelState::MOVE_TO_FRONT, [this]() {
    return robot()->getDistance(world_model()->ball.pos) > 1.0;
  });

  addStateFunction(
    StealBallVelState::STEAL,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command.disableBallAvoidance();
      command.disableCollisionAvoidance();

      auto theirs = world_model()->theirs.getAvailableRobots() | std::views::filter([](const auto& r) { return r != nullptr; });
      const auto pos = robot()->pose.pos;
      if (not theirs.empty()) {
        auto [ball_holder, distance] =
          world_model()->getNearestRobotWithDistanceFromPoint(world_model()->ball.pos, theirs);

        // ボールにあたりに行くときの傾き
        using boost::math::constants::degree;
        const double ANGLE_OFFSET = 30. * degree<double>();

        const double HYSTERESIS = 0.3;
        double diff =
          getVerticalVec(getNormVec(ball_holder->pose.theta)).dot(pos - ball_holder->pose.pos);
        if (std::abs(diff) > HYSTERESIS) {
          double angle1 = getAngle(ball_holder->pose.pos - pos) + ANGLE_OFFSET;
          double angle2 = getAngle(ball_holder->pose.pos - pos) - ANGLE_OFFSET;
          double angle = getAngle(world_model()->ball.pos - pos);
          if (std::abs(getAngleDiff(angle1, angle)) > std::abs(getAngleDiff(angle2, angle))) {
            // OFFSET -
            steal_to_left = false;
          } else {
            // OFFSET +
            steal_to_left = true;
          }
        }

        command.setTargetTheta(normalizeAngle(
          getAngle(ball_holder->pose.pos - pos) + ANGLE_OFFSET * (steal_to_left ? 1. : -1.)));
        if (robot()->getDistance(world_model()->ball.pos) > 0.12) {
          // ボールに向かって突っ込む
          command.setVelocity(
            (world_model()->ball.pos - pos).normalized() * getParameter<double>("突入速度"));
          command.dribble(0.0);
        } else {
          // 回転半径：
          const double RADIUS = 0.085 * 2;
          using boost::math::constants::half_pi;
          command.setVelocity(
            getParameter<double>("振り切り速度") *
            getNormVec(
              getAngle(ball_holder->pose.pos - pos) +
              (steal_to_left ? half_pi<double>() : -half_pi<double>())));
          if (ball_holder->getDistance(world_model()->ball.pos) > (0.085 + 0.05)) {
            command.kickWithChip(0.5);
          } else {
            command.dribble(0.3);
          }
        }
      }
      return Status::RUNNING;
    });
}
}  // namespace crane::skills
