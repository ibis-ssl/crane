// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/penalty_kick.hpp>

namespace crane::skills
{
PenaltyKick::PenaltyKick(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<PenaltyKickState>("PenaltyKick", id, wm, PenaltyKickState::PREPARE)
{
  setParameter("start_from_kick", false);
  setParameter("prepare_margin", 0.6);
  addStateFunction(
    PenaltyKickState::PREPARE,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      Point target = world_model->ball.pos;
      auto margin = getParameter<double>("prepare_margin");
      target.x() += world_model->getOurGoalCenter().x() > 0 ? margin : -margin;
      command->setTargetPosition(target);
      command->lookAtBall();
      command->disableRuleAreaAvoidance();
      return Status::RUNNING;
    });

  addTransition(PenaltyKickState::PREPARE, PenaltyKickState::KICK, [this]() {
    if (getParameter<bool>("start_from_kick")) {
      return true;
    } else {
      return world_model->play_situation.getSituationCommandID() ==
             crane_msgs::msg::PlaySituation::OUR_PENALTY_START;
    }
  });
  addStateFunction(
    PenaltyKickState::KICK,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not start_ball_point) {
        start_ball_point = world_model->ball.pos;
      }

      auto [best_angle, goal_angle_width] =
        world_model->getLargestGoalAngleRangeFromPoint(world_model->ball.pos);
      Point best_target = world_model->ball.pos + getNormVec(best_angle) * 0.5;
      visualizer->addPoint(best_target.x(), best_target.y(), 1, "red", 1.0, "best_target");

      // 経由ポイント
      Point intermediate_point =
        world_model->ball.pos + (world_model->ball.pos - best_target).normalized() * 0.2;
      visualizer->addPoint(
        intermediate_point.x(), intermediate_point.y(), 1, "red", 1.0, "intermediate_point");

      double dot = (robot->pose.pos - world_model->ball.pos)
                     .normalized()
                     .dot((world_model->ball.pos - best_target).normalized());
      double target_theta = getAngle(best_target - world_model->ball.pos);
      // ボールと敵ゴールの延長線上にいない && 角度があってないときは，中間ポイントを経由
      if (dot < 0.9 || std::abs(getAngleDiff(target_theta, robot->pose.theta)) > 0.1) {
        command->setTargetPosition(intermediate_point);
        command->enableCollisionAvoidance();
      } else {
        command->setTargetPosition(world_model->ball.pos);
        command->kickStraight(0.3).disableCollisionAvoidance();
        command->enableCollisionAvoidance();
        command->disableBallAvoidance();
      }

      command->setTargetTheta(target_theta);
      command->disableRuleAreaAvoidance();
      return Status::RUNNING;
    });

  addTransition(PenaltyKickState::KICK, PenaltyKickState::DONE, [this]() {
    return world_model->point_checker.isDefenseArea(world_model->ball.pos);
  });

  addStateFunction(
    PenaltyKickState::DONE,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->stopHere();
      return Status::RUNNING;
    });
}
}  // namespace crane::skills
