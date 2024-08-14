// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/steal_ball.hpp>

namespace crane::skills
{
StealBall::StealBall(RobotCommandWrapperBase::SharedPtr & base)
: SkillBaseWithState<StealBallState>("StealBall", base, StealBallState::MOVE_TO_FRONT)
{
  // ボールを奪う方法
  // front: 正面からドリブラーでボールを奪う
  // side: 横から押し出すようにボールを奪う
  setParameter("steal_method", std::string("side"));
  setParameter("kicker_power", 0.4);
  addStateFunction(
    StealBallState::MOVE_TO_FRONT,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      // ボールの正面に移動
      // 到着判定すると遅くなるので、敵ロボットにボールが隠されていなかったら次に行ってもいいかも
      auto theirs = world_model()->theirs.getAvailableRobots();
      if (not theirs.empty()) {
        auto [ball_holder, distance] =
          world_model()->getNearestRobotWithDistanceFromPoint(world_model()->ball.pos, theirs);
        Point target_pos = world_model()->ball.pos + getNormVec(ball_holder->pose.theta) * 0.3;
        command.setTargetPosition(target_pos);
        command.lookAtBallFrom(target_pos);
        if ((robot()->pose.pos - target_pos).norm() < 0.2) {
          skill_state = Status::SUCCESS;
        } else {
          skill_state = Status::RUNNING;
        }
        return skill_state;
      } else {
        return Status::RUNNING;
      }
    });

  // 正面に移動したら突っ込んでボールを奪う
  addTransition(StealBallState::MOVE_TO_FRONT, StealBallState::STEAL, [this]() {
    return skill_state == Status::SUCCESS;
  });

  addStateFunction(
    StealBallState::STEAL,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command.disableBallAvoidance();
      command.disableCollisionAvoidance();
      const auto method = getParameter<std::string>("steal_method");
      auto their_frontier = world_model()->getNearestRobotWithDistanceFromPoint(
        world_model()->ball.pos, world_model()->theirs.getAvailableRobots());
      if (method == "front") {
        command.setTargetPosition(
          world_model()->ball.pos, getAngle(world_model()->ball.pos - robot()->pose.pos));
        command.dribble(0.5);
      } else if (method == "side") {
        command.setTargetPosition(
          world_model()->ball.pos, getAngle(world_model()->ball.pos - robot()->pose.pos));
        command.setTargetTheta(getAngle(world_model()->ball.pos - robot()->pose.pos));
        if (robot()->getDistance(world_model()->ball.pos) < (0.085 - 0.040)) {
          command.setDribblerTargetPosition(
            world_model()->ball.pos +
            getVerticalVec(world_model()->ball.pos - robot()->pose.pos) * 0.7);
          // ロボット半径より近くに来れば急回転して刈り取れる
           command.setTargetTheta(
            getAngle(world_model()->ball.pos - robot()->pose.pos) + M_PI / 2);
        } else {
          command.setDribblerTargetPosition(world_model()->ball.pos);
        }
        if (
          world_model()->getTheirFrontier().has_value() &&
          robot()->getDistance(world_model()->ball.pos) <
            world_model()->getTheirFrontier()->robot->getDistance(world_model()->ball.pos)) {
          command.kickWithChip(0.5);
        }
      }
      return Status::RUNNING;
    });
}
}  // namespace crane::skills
