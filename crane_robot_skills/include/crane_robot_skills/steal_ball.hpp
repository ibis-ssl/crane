// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_
#define CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/simple_attacker.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>
#include <string>

namespace crane::skills
{
enum class StealBallState {
  MOVE_TO_FRONT,
  STEAL,
  PASS,
};
class StealBall : public SkillBase<StealBallState>
{
public:
  explicit StealBall(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
  : SkillBase<StealBallState>("StealBall", id, wm, StealBallState::MOVE_TO_FRONT)
  {
    // ボールを奪う方法
    // front: 正面からドリブラーでボールを奪う
    // side: 横から押し出すようにボールを奪う
    setParameter("steal_method", std::string("side"));
    addStateFunction(
      StealBallState::MOVE_TO_FRONT,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        // ボールの正面に移動
        // 到着判定すると遅くなるので、敵ロボットにボールが隠されていなかったら次に行ってもいいかも
        auto [ball_holder, distance] = world_model->getNearestRobotsWithDistanceFromPoint(
          world_model->ball.pos, world_model->theirs.getAvailableRobots());
        Point target_pos = world_model->ball.pos + getNormVec(ball_holder->pose.theta) * 0.3;
        command->setTargetPosition(target_pos);
        command->lookAtBallFrom(target_pos);
        if ((robot->pose.pos - target_pos).norm() < 0.2) {
          skill_state = Status::SUCCESS;
        } else {
          skill_state = Status::RUNNING;
        }
        return skill_state;
      });

    addTransition(StealBallState::MOVE_TO_FRONT, StealBallState::STEAL, [this]() {
      return skill_state == Status::SUCCESS;
    });

    addTransition(StealBallState::MOVE_TO_FRONT, StealBallState::PASS, [this]() {
      auto [their_attacker, their_distance] = world_model->getNearestRobotsWithDistanceFromPoint(
        world_model->ball.pos, world_model->theirs.getAvailableRobots());
      double our_distance = robot->getDistance(world_model->ball.pos);
      return our_distance < their_distance - 0.2;
    });

    addStateFunction(
      StealBallState::STEAL,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        command->disableBallAvoidance();
        command->disableCollisionAvoidance();
        const auto method = getParameter<std::string>("steal_method");
        if (method == "front") {
          command->setDribblerTargetPosition(world_model->ball.pos);
          command->dribble(0.5);
        } else if (method == "side") {
          command->setDribblerTargetPosition(world_model->ball.pos);
          if (robot->getDistance(world_model->ball.pos) < (0.085 + 0.005)) {
            // ロボット半径より近くに来れば急回転して刈り取れる
            command->setTargetTheta(getAngle(world_model->ball.pos - robot->pose.pos) + M_PI / 2);
          } else {
            command->setTargetTheta(getAngle(world_model->ball.pos - robot->pose.pos));
          }
        }
        return Status::RUNNING;
      });

    addTransition(StealBallState::STEAL, StealBallState::PASS, [this]() {
      auto [their_attacker, their_distance] = world_model->getNearestRobotsWithDistanceFromPoint(
        world_model->ball.pos, world_model->theirs.getAvailableRobots());
      double our_distance = robot->getDistance(world_model->ball.pos);
      return our_distance < their_distance - 0.2;
    });

    addStateFunction(
      StealBallState::PASS,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        if (attacker_skill == nullptr) {
          attacker_skill = std::make_shared<skills::SimpleAttacker>(robot->id, world_model);
          attacker_skill->setCommander(command);
        }
        auto [target_bot, distance] = world_model->getNearestRobotsWithDistanceFromPoint(
          world_model->getTheirGoalCenter(), world_model->ours.getAvailableRobots(robot->id));
        attacker_skill->setParameter("receiver_id", target_bot->id);
        std::cout << "PASS" << std::endl;
        return attacker_skill->run(visualizer);
      });

    addTransition(StealBallState::PASS, StealBallState::MOVE_TO_FRONT, [this]() {
      auto [their_attacker, their_distance] = world_model->getNearestRobotsWithDistanceFromPoint(
        world_model->ball.pos, world_model->theirs.getAvailableRobots());
      double our_distance = robot->getDistance(world_model->ball.pos);
      return our_distance > their_distance;
    });
  }

  void print(std::ostream & os) const override { os << "[StealBall]"; }

  Status skill_state = Status::RUNNING;

  std::shared_ptr<skills::SimpleAttacker> attacker_skill = nullptr;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_
