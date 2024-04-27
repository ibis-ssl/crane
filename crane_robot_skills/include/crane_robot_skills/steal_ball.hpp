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
  INTERCEPT,
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
    setParameter("kicker_power", 0.4);
    addStateFunction(
      StealBallState::MOVE_TO_FRONT,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        // ボールの正面に移動
        // 到着判定すると遅くなるので、敵ロボットにボールが隠されていなかったら次に行ってもいいかも
        auto theirs = world_model->theirs.getAvailableRobots();
        if (not theirs.empty()) {
          auto [ball_holder, distance] =
            world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, theirs);
          Point target_pos = world_model->ball.pos + getNormVec(ball_holder->pose.theta) * 0.3;
          command->setTargetPosition(target_pos);
          command->lookAtBallFrom(target_pos);
          if ((robot->pose.pos - target_pos).norm() < 0.2) {
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

    // 敵よりもボールに近い場合はパス
    addTransition(StealBallState::MOVE_TO_FRONT, StealBallState::PASS, [this]() {
      auto theirs = world_model->theirs.getAvailableRobots();
      if (not theirs.empty()) {
        auto [their_attacker, their_distance] =
          world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, theirs);
        double our_distance = robot->getDistance(world_model->ball.pos);
        return our_distance < their_distance - 0.2;
      } else {
        return true;
      }
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

    // 敵よりもボールに近い場合はパス
    addTransition(StealBallState::STEAL, StealBallState::PASS, [this]() {
      auto theirs = world_model->theirs.getAvailableRobots();
      if (not theirs.empty()) {
        auto [their_attacker, their_distance] =
          world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, theirs);
        double our_distance = robot->getDistance(world_model->ball.pos);
        return our_distance < their_distance - 0.2;
      } else {
        return true;
      }
    });

    addStateFunction(
      StealBallState::PASS,
      [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
        if (attacker_skill == nullptr) {
          attacker_skill = std::make_shared<skills::SimpleAttacker>(robot->id, world_model);
          attacker_skill->setCommander(command);
        }
        auto ours = world_model->ours.getAvailableRobots(robot->id);
        ours.erase(std::remove_if(ours.begin(), ours.end(), [this](auto e) {
          return e->getDistance(world_model->getTheirGoalCenter()) >
                 robot->getDistance(world_model->getTheirGoalCenter());
        }), ours.end());
        if (not ours.empty()) {
          auto [target_bot, distance] = world_model->getNearestRobotsWithDistanceFromPoint(
            world_model->getTheirGoalCenter(), ours);
          attacker_skill->setParameter("receiver_id", target_bot->id);
        }
        return attacker_skill->run(visualizer);
      });

    addTransition(StealBallState::PASS, StealBallState::MOVE_TO_FRONT, [this]() {
      auto theirs = world_model->theirs.getAvailableRobots();
      if (theirs.empty()) {
        return false;
      } else {
        auto [their_attacker, their_distance] =
          world_model->getNearestRobotsWithDistanceFromPoint(world_model->ball.pos, theirs);
        double our_distance = robot->getDistance(world_model->ball.pos);
        return our_distance > their_distance;
      }
    });

    addTransition(StealBallState::PASS, StealBallState::INTERCEPT, [this]() {
      return world_model->ball.vel.norm() > 0.5;
    });

    addTransition(StealBallState::STEAL, StealBallState::INTERCEPT, [this]() {
      return world_model->ball.vel.norm() > 0.5;
    });

    addTransition(StealBallState::MOVE_TO_FRONT, StealBallState::INTERCEPT, [this]() {
      return world_model->ball.vel.norm() > 0.5;
    });

    addStateFunction(StealBallState::INTERCEPT, [this](const ConsaiVisualizerWrapper::SharedPtr &) {
      std::cout << "Intercept" << std::endl;
      Segment ball_line{
        world_model->ball.pos, world_model->ball.pos + world_model->ball.vel.normalized() * 10.0};

      ClosestPoint result;
      bg::closest_point(robot->pose.pos, ball_line, result);

      // ゴールとボールの中間方向を向く
      auto [goal_angle, width] =
        world_model->getLargestGoalAngleRangeFromPoint(result.closest_point);
      auto to_goal = getNormVec(goal_angle);
      auto to_ball = (world_model->ball.pos - result.closest_point).normalized();
      double intermediate_angle = getAngle(2 * to_goal + to_ball);
      command->setTargetTheta(intermediate_angle);
      command->liftUpDribbler();
      command->kickStraight(getParameter<double>("kicker_power"));
      command->setDribblerTargetPosition(result.closest_point);

      return Status::RUNNING;
    });

    addTransition(StealBallState::INTERCEPT, StealBallState::MOVE_TO_FRONT, [this]() {
      return world_model->ball.vel.norm() < 0.3;
    });
  }

  void print(std::ostream & os) const override { os << "[StealBall]"; }

  Status skill_state = Status::RUNNING;

  std::shared_ptr<skills::SimpleAttacker> attacker_skill = nullptr;
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__STEAL_BALL_HPP_
