// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/goal_kick.hpp>
#include <crane_robot_skills/penalty_kick.hpp>

namespace crane::skills
{
void PenaltyKick::initialize()
{
  // SimpleAIでテストするためのパラメータ
  setParameter("start_from_kick", false);
  setParameter("prepare_margin", 0.6);
  addStateFunction(PenaltyKickState::PREPARE, [this]() -> Status {
    Point target = world_model()->ball.pos;
    auto margin = getParameter<double>("prepare_margin");
    target.x() += world_model()->getOurGoalCenter().x() > 0 ? margin : -margin;
    command->setTargetPosition(target);
    command->lookAtBall();
    command->disableAnyAreaAvoidance();
    return Status::RUNNING;
  });

  addTransition(PenaltyKickState::PREPARE, PenaltyKickState::KICK, [this]() {
    if (getParameter<bool>("start_from_kick")) {
      return true;
    } else {
      return world_model()->getMsg().play_situation.command.value ==
             crane_msgs::msg::PlaySituation::OUR_PENALTY_START;
    }
  });
  addStateFunction(PenaltyKickState::KICK, [this]() -> Status {
    if (not start_ball_point) {
      start_ball_point = world_model()->ball.pos;
    }

    double minimum_angle_accuracy = 2.0 * M_PI / 180.;
    double best_angle = GoalKick::getBestAngleToShootFromPoint(
      minimum_angle_accuracy, world_model()->ball.pos, world_model(), visualizer);
    Point best_target = world_model()->ball.pos + getNormVec(best_angle) * 0.5;
    visualizer->circle().center(best_target).radius(0.1).stroke("red").strokeWidth(10).build();

    kick_skill.setParameter("target", best_target);

    double dist_ball_goal =
      std::abs(world_model()->getTheirGoalCenter().x() - world_model()->ball.pos.x());
    if (dist_ball_goal < world_model()->getDefenseHeight() + 2.0) {
      kick_skill.setParameter("kick_power", 0.8);
    } else {
      kick_skill.setParameter("kick_power", 0.4);
    }
    kick_skill.run();
    command->disableAnyAreaAvoidance();
    return Status::RUNNING;
  });

  addTransition(PenaltyKickState::KICK, PenaltyKickState::DONE, [this]() {
    return world_model()->point_checker.isPenaltyArea(world_model()->ball.pos);
  });

  addStateFunction(PenaltyKickState::DONE, [this]() -> Status {
    command->stopHere();
    return Status::RUNNING;
  });
}
}  // namespace crane::skills
