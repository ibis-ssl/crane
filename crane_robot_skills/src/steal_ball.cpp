// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/steal_ball.hpp>

namespace crane::skills
{
StealBall::StealBall(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
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
    StealBallState::STEAL, [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->disableBallAvoidance();
      command->disableCollisionAvoidance();
      const auto method = getParameter<std::string>("steal_method");
      if (method == "front") {
        command->setDribblerTargetPosition(world_model->ball.pos);
        command->dribble(0.5);
      } else if (method == "side") {
        command->setDribblerTargetPosition(world_model->ball.pos);
        if (robot->getDistance(world_model->ball.pos) < (0.085 + 0.000)) {
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
    StealBallState::PASS, [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (attacker_skill == nullptr) {
        attacker_skill = std::make_shared<skills::SimpleAttacker>(robot->id, world_model);
        attacker_skill->setCommander(command);
      }
      auto ours = world_model->ours.getAvailableRobots(robot->id);
      ours.erase(
        std::remove_if(
          ours.begin(), ours.end(),
          [this](auto e) {
            return e->getDistance(world_model->getTheirGoalCenter()) >
                   robot->getDistance(world_model->getTheirGoalCenter());
          }),
        ours.end());
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
    Point vel_seg = world_model->ball.vel * 5.0;
    if (vel_seg.norm() < 1.0) {
      vel_seg = vel_seg.normalized() * 1.0;
    }
    Segment ball_line{world_model->ball.pos, world_model->ball.pos + vel_seg};

    Point across_point = [&]() {
      const double OFFSET = 0.3;
      const double X = world_model->field_size.x() / 2.0 - OFFSET;
      const double Y = world_model->field_size.y() / 2.0 - OFFSET;

      Segment seg1{Point(X, Y), Point(X, -Y)};
      Segment seg2{Point(-X, Y), Point(-X, -Y)};
      Segment seg3{Point(Y, X), Point(-Y, X)};
      Segment seg4{Point(Y, -X), Point(-Y, -X)};
      std::vector<Point> intersections;
      if (bg::intersection(ball_line, seg1, intersections); not intersections.empty()) {
        return intersections.front();
      } else if (bg::intersection(ball_line, seg2, intersections); not intersections.empty()) {
        return intersections.front();
      } else if (bg::intersection(ball_line, seg3, intersections); not intersections.empty()) {
        return intersections.front();
      } else if (bg::intersection(ball_line, seg4, intersections); not intersections.empty()) {
        return intersections.front();
      } else {
        return ball_line.second;
      }
    }();

    // ゴールとボールの中間方向を向く
    auto [goal_angle, width] = world_model->getLargestGoalAngleRangeFromPoint(across_point);
    auto to_goal = getNormVec(goal_angle);
    auto to_ball = (world_model->ball.pos - across_point).normalized();
    double intermediate_angle = getAngle(2 * to_goal + to_ball);
    command->setTargetTheta(intermediate_angle);
    command->liftUpDribbler();
    command->kickStraight(getParameter<double>("kicker_power"));
    command->setDribblerTargetPosition(across_point);

    return Status::RUNNING;
  });

  addTransition(StealBallState::INTERCEPT, StealBallState::MOVE_TO_FRONT, [this]() {
    return world_model->ball.vel.norm() < 0.3;
  });
}
}  // namespace crane::skills
