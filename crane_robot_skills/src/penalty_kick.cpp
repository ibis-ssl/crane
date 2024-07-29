// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/penalty_kick.hpp>

namespace crane::skills
{
PenaltyKick::PenaltyKick(RobotCommandWrapperBase::SharedPtr & base)
: SkillBaseWithState<PenaltyKickState, RobotCommandWrapperPosition>(
    "PenaltyKick", base, PenaltyKickState::PREPARE),
  start_ball_point(getContextReference<std::optional<Point>>("start_ball_point", std::nullopt)),
  kick_skill(base)
{
  // SimpleAIでテストするためのパラメータ
  setParameter("start_from_kick", false);
  setParameter("prepare_margin", 0.6);
  addStateFunction(
    PenaltyKickState::PREPARE,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      Point target = world_model()->ball.pos;
      auto margin = getParameter<double>("prepare_margin");
      target.x() += world_model()->getOurGoalCenter().x() > 0 ? margin : -margin;
      command.setTargetPosition(target);
      command.lookAtBall();
      command.disableRuleAreaAvoidance();
      return Status::RUNNING;
    });

  addTransition(PenaltyKickState::PREPARE, PenaltyKickState::KICK, [this]() {
    if (getParameter<bool>("start_from_kick")) {
      return true;
    } else {
      return world_model()->play_situation.getSituationCommandID() ==
             crane_msgs::msg::PlaySituation::OUR_PENALTY_START;
    }
  });
  addStateFunction(
    PenaltyKickState::KICK,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not start_ball_point) {
        start_ball_point = world_model()->ball.pos;
      }

      auto [best_angle, goal_angle_width] =
        world_model()->getLargestGoalAngleRangeFromPoint(world_model()->ball.pos);
      Point best_target = world_model()->ball.pos + getNormVec(best_angle) * 0.5;
      visualizer->addPoint(best_target.x(), best_target.y(), 1, "red", 1.0, "best_target");

      kick_skill.setParameter("target", best_target);
      kick_skill.setParameter("kick_power", 0.3);
      kick_skill.run(visualizer);
      command.disableRuleAreaAvoidance();
      return Status::RUNNING;
    });

  addTransition(PenaltyKickState::KICK, PenaltyKickState::DONE, [this]() {
    return world_model()->point_checker.isPenaltyArea(world_model()->ball.pos);
  });

  addStateFunction(
    PenaltyKickState::DONE,
    [this]([[maybe_unused]] const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command.stopHere();
      return Status::RUNNING;
    });
}
}  // namespace crane::skills
