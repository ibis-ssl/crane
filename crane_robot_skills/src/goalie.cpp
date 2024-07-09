// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/goalie.hpp>

namespace crane::skills
{
Goalie::Goalie(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<>("Goalie", id, wm, DefaultStates::DEFAULT)
{
  setParameter("run_inplay", true);
  setParameter("block_distance", 1.0);
  addStateFunction(
    DefaultStates::DEFAULT,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      auto situation = world_model->play_situation.getSituationCommandID();
      if (getParameter<bool>("run_inplay")) {
        situation = crane_msgs::msg::PlaySituation::OUR_INPLAY;
      }

      auto cmd = std::dynamic_pointer_cast<RobotCommandWrapperPosition>(command);
      switch (situation) {
        case crane_msgs::msg::PlaySituation::HALT:
          phase = "HALT, stop here";
          cmd->stopHere();
          break;
        case crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION:
          [[fallthrough]];
        case crane_msgs::msg::PlaySituation::THEIR_PENALTY_START:
          phase = "ペナルティキック";
          inplay(cmd, false, visualizer);
          break;
        default:
          inplay(cmd, true, visualizer);
          break;
      }
      visualizer->addPoint(robot->pose.pos.x(), robot->pose.pos.y(), 0, "white", 1., phase);
      return Status::RUNNING;
    });
}

void Goalie::emitBallFromPenaltyArea(
  RobotCommandWrapperPosition::SharedPtr & command,
  const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  Point ball = world_model->ball.pos;
  // パスできるロボットのリストアップ
  auto passable_robot_list = world_model->ours.getAvailableRobots(command->robot->id);
  passable_robot_list.erase(
    std::remove_if(
      passable_robot_list.begin(), passable_robot_list.end(),
      [&](const RobotInfo::SharedPtr & r) {
        if (
          std::abs(r->pose.pos.x() - world_model->getOurGoalCenter().x()) <
          world_model->getDefenseHeight()) {
          // ゴールラインに近いロボットは除外
          return true;
        } else if (world_model->getDistanceFromRobotToBall(r->getID()) < 0.5) {
          // ボールに近いロボットは除外
          return true;
        } else {
          return false;
        }
      }),
    passable_robot_list.end());

  Point pass_target = [&]() {
    if (not passable_robot_list.empty()) {
      // TODO(HansRobo): いい感じのロボットを選ぶようにする
      return passable_robot_list.front()->pose.pos;
    } else {
      return world_model->getTheirGoalCenter();
    }
  }();

  visualizer->addLine(ball, pass_target, 1, "blue");

  Point intermediate_point = ball + (ball - pass_target).normalized() * 0.2f;
  double angle_ball_to_target = getAngle(pass_target - ball);
  double dot = (world_model->ball.pos - command->robot->pose.pos)
                 .normalized()
                 .dot((pass_target - world_model->ball.pos).normalized());
  // ボールと目標の延長線上にいない && 角度があってないときは，中間ポイントを経由
  if (dot < 0.9 || std::abs(getAngleDiff(angle_ball_to_target, command->robot->pose.theta)) > 0.1) {
    command->setTargetPosition(intermediate_point).enableCollisionAvoidance().enableBallAvoidance();
  } else {
    command->setTargetPosition(world_model->ball.pos)
      .kickWithChip(1.0)
      .disableCollisionAvoidance()
      .enableCollisionAvoidance()
      .disableBallAvoidance();
  }
  command->setTargetTheta(getAngle(pass_target - command->robot->pose.pos))
    .disableGoalAreaAvoidance()
    .disableRuleAreaAvoidance();
}

void Goalie::inplay(
  RobotCommandWrapperPosition::SharedPtr & command, bool enable_emit,
  const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  std::cout << "Goalie::inplay" << std::endl;
  auto goals = world_model->getOurGoalPosts();
  const auto & ball = world_model->ball;
  std::cout << "Goalie::inplay" << std::endl;
  // シュートチェック
  Segment goal_line(goals.first, goals.second);
  Segment ball_line(ball.pos, ball.pos + ball.vel.normalized() * 20.f);
  auto intersections = getIntersections(ball_line, Segment{goals.first, goals.second});
  std::cout << "Goalie::inplay" << std::endl;
  command->setTerminalVelocity(0.0);
//    .disableGoalAreaAvoidance()
//    .disableBallAvoidance()
//    .disableRuleAreaAvoidance();

  std::cout << "Goalie::inplay2" << std::endl;
  if (not intersections.empty() && world_model->ball.vel.norm() > 0.3f) {
    // シュートブロック
    phase = "シュートブロック";
    auto result = getClosestPointAndDistance(ball_line, command->robot->pose.pos);
    auto target = [&]() {
      if (not world_model->point_checker.isFieldInside(result.closest_point)) {
        // フィールド外（=ゴール内）でのセーブは避ける
        return intersections.front();
      } else {
        return result.closest_point;
      }
    }();

    command->setTargetPosition(target).lookAtBallFrom(target);
    if (command->robot->getDistance(target) > 0.05) {
      // なりふり構わず爆加速
      command->setTerminalVelocity(2.0).setMaxAcceleration(5.0).setMaxVelocity(5.0);
    }
  } else {
    if (
      world_model->ball.isStopped() && world_model->point_checker.isFriendPenaltyArea(ball.pos) &&
      enable_emit) {
      // ボールが止まっていて，味方ペナルティエリア内にあるときは，ペナルティエリア外に出す
      phase = "ボール排出";
      emitBallFromPenaltyArea(command, visualizer);
    } else {
      phase = "";
      const double BLOCK_DIST = getParameter<double>("block_distance");
      phase += "ボールを待ち受ける";
      if (std::signbit(world_model->ball.pos.x()) == std::signbit(world_model->goal.x())) {
        phase += " (自コート警戒モード)";
        Segment ball_prediction_2s(ball.pos, ball.pos + ball.vel * 2.0);
        auto [next_their_attacker, distance] = [&]() {
          std::shared_ptr<RobotInfo> nearest_enemy = nullptr;
          double min_distance = 1000000.0;
          for (const auto & enemy : world_model->theirs.getAvailableRobots()) {
            double dist = bg::distance(enemy->pose.pos, ball_prediction_2s);
            if (dist < min_distance) {
              Vector2 ball_to_enemy = (enemy->pose.pos - ball.pos).normalized();
              Vector2 ball_direction = ball.vel.normalized();
              //  ボールの進行方向のロボットのみ反映
              if (ball_to_enemy.dot(ball_direction) > 0.0) {
                min_distance = dist;
                nearest_enemy = enemy;
              }
            }
          }
          return std::make_pair(nearest_enemy, min_distance);
        }();

        Point goal_center = world_model->getOurGoalCenter();
        goal_center << goals.first.x() - std::clamp(goals.first.x(), -0.1, 0.1), 0.0f;

        if (not world_model->point_checker.isFieldInside(ball.pos)) {
          phase += "(範囲外なので正面に構える)";
          command->setTargetPosition(goal_center).lookAt(Point(0, 0));
        } else {
          Point threat_point;
          if (distance < 2.0) {
            phase += "(敵のパス先警戒モード)";
            auto result =
              getClosestPointAndDistance(ball_prediction_2s, next_their_attacker->pose.pos);
            threat_point = result.closest_point;
          } else {
            phase += "(とりあえず0.5s先を警戒モード)";
            threat_point = ball.pos + ball.vel * 0.5;
          }
          Point weak_point = [&]() {
            auto [angle, interval] = world_model->getLargestOurGoalAngleRangeFromPoint(
              threat_point, world_model->ours.getAvailableRobots(world_model->getOurGoalieId()));
            Segment expected_ball_line(threat_point, threat_point + getNormVec(angle) * 10);
            Segment goal_line(goals.first, goals.second);
            auto intersections = getIntersections(expected_ball_line, goal_line);
            if (intersections.empty()) {
              return goal_center;
            } else {
              return intersections.front();
            }
          }();

          Point wait_point = weak_point + (threat_point - weak_point).normalized() * BLOCK_DIST;

          command->setTargetPosition(wait_point).lookAtBallFrom(wait_point);
          if (command->robot->getDistance(wait_point) > 0.05) {
            // なりふり構わず爆加速
            command->setTerminalVelocity(2.0).setMaxAcceleration(5.0).setMaxVelocity(5.0);
          }
        }
      }
    }
  }
}
}  // namespace crane::skills
