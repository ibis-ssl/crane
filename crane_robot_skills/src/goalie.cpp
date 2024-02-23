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
  addStateFunction(
    DefaultStates::DEFAULT, [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      auto situation = world_model->play_situation.getSituationCommandID();
      if (getParameter<bool>("run_inplay")) {
        situation = crane_msgs::msg::PlaySituation::INPLAY;
      }
      switch (situation) {
        case crane_msgs::msg::PlaySituation::HALT:
          phase = "HALT, stop here";
          command->stopHere();
          break;
        case crane_msgs::msg::PlaySituation::THEIR_PENALTY_PREPARATION:
          [[fallthrough]];
        case crane_msgs::msg::PlaySituation::THEIR_PENALTY_START:
          phase = "ペナルティキック";
          inplay(command, false);
          break;
        default:
          inplay(command, true);
          break;
      }
      return Status::RUNNING;
    });
}

void Goalie::emitBallFromPenaltyArea(RobotCommandWrapper::SharedPtr & command)
{
  auto ball = world_model->ball.pos;
  // パスできるロボットのリストアップ
  auto passable_robot_list = world_model->ours.getAvailableRobots(command->robot->id);
  passable_robot_list.erase(
    std::remove_if(
      passable_robot_list.begin(), passable_robot_list.end(),
      [&](const RobotInfo::SharedPtr & r) {
        // 敵に塞がれていたら除外
        Segment ball_to_robot_line(ball, r->pose.pos);
        for (const auto & enemy : world_model->theirs.getAvailableRobots()) {
          auto dist = bg::distance(ball_to_robot_line, enemy->pose.pos);
          if (dist < 0.2) {
            return true;
          }
        }
        return false;
      }),
    passable_robot_list.end());

  Point pass_target = [&]() {
    if (not passable_robot_list.empty()) {
      // TODO(HansRobo): いい感じのロボットを選ぶようにする
      return passable_robot_list.front()->pose.pos;
    } else {
      return Point{0, 0};
    }
  }();

  Point intermediate_point = ball + (ball - pass_target).normalized() * 0.2f;
  double angle_ball_to_target = getAngle(pass_target - ball);
  double dot = (world_model->ball.pos - command->robot->pose.pos)
                 .normalized()
                 .dot((pass_target - world_model->ball.pos).normalized());
  // ボールと目標の延長線上にいない && 角度があってないときは，中間ポイントを経由
  if (
    dot < 0.95 || std::abs(getAngleDiff(angle_ball_to_target, command->robot->pose.theta)) > 0.05) {
    command->setTargetPosition(intermediate_point);
    command->enableCollisionAvoidance();
  } else {
    command->setTargetPosition(world_model->ball.pos);
    command->kickWithChip(0.4).disableCollisionAvoidance();
    command->enableCollisionAvoidance();
    command->disableBallAvoidance();
  }
  command->setTargetTheta(getAngle(pass_target - ball));
  command->disableGoalAreaAvoidance();
}

void Goalie::inplay(RobotCommandWrapper::SharedPtr & command, bool enable_emit)
{
  auto goals = world_model->getOurGoalPosts();
  auto ball = world_model->ball.pos;
  // シュートチェック
  Segment goal_line(goals.first, goals.second);
  Segment ball_line(ball, ball + world_model->ball.vel.normalized() * 20.f);
  std::vector<Point> intersections;
  bg::intersection(ball_line, Segment{goals.first, goals.second}, intersections);
  if (not intersections.empty() && world_model->ball.vel.norm() > 0.5f) {
    // シュートブロック
    phase = "シュートブロック";
    ClosestPoint result;
    bg::closest_point(ball_line, command->robot->pose.pos, result);
    command->setTargetPosition(result.closest_point);
    command->setTargetTheta(getAngle(-world_model->ball.vel));
  } else {
    if (world_model->ball.isStopped() && world_model->isFriendDefenseArea(ball) && enable_emit) {
      // ボールが止まっていて，味方ペナルティエリア内にあるときは，ペナルティエリア外に出す
      phase = "ボール排出";
      emitBallFromPenaltyArea(command);
    } else {
      phase = "";
      const double BLOCK_DIST = 0.15;
      // 範囲外のときは正面に構える
      if (not world_model->isFieldInside(ball)) {
        phase += "正面で";
        ball << 0, 0;
      }

      phase = "ボールを待ち受ける";
      Point goal_center = world_model->getOurGoalCenter();
      goal_center << goals.first.x(), 0.0f;
      command->setTargetPosition(goal_center + (ball - goal_center).normalized() * BLOCK_DIST);
      command->lookAtBall();
    }
  }
}
}  // namespace crane::skills
