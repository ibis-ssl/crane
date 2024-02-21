// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/single_ball_placement.hpp>

namespace crane::skills
{

SingleBallPlacement::SingleBallPlacement(uint8_t id, const std::shared_ptr<WorldModelWrapper> & wm)
: SkillBase<SingleBallPlacementStates>(
    "SingleBallPlacement", id, wm, SingleBallPlacementStates::GO_OVER_BALL)
{
  setParameter("placement_x", 0.);
  setParameter("placement_y", 0.);
  addStateFunction(
    SingleBallPlacementStates::GO_OVER_BALL,
    [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      if (not go_over_ball) {
        go_over_ball = std::make_shared<GoOverBall>(robot->id, world_model);
        go_over_ball->setParameter("next_target_x", getParameter<double>("placement_x"));
        go_over_ball->setParameter("next_target_y", getParameter<double>("placement_y"));
        go_over_ball->setParameter("margin", 0.4);
      }

      skill_status = go_over_ball->run(visualizer);

      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::GO_OVER_BALL, SingleBallPlacementStates::CONTACT_BALL,
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(
    SingleBallPlacementStates::CONTACT_BALL,
    [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      if (not get_ball_contact) {
        get_ball_contact = std::make_shared<GetBallContact>(robot->id, world_model);
      }

      skill_status = get_ball_contact->run(visualizer);

      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::CONTACT_BALL, SingleBallPlacementStates::MOVE_TO_TARGET,
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(
    SingleBallPlacementStates::MOVE_TO_TARGET,
    [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      if (not move_with_ball) {
        move_with_ball = std::make_shared<MoveWithBall>(robot->id, world_model);
        move_with_ball->setParameter("target_x", getParameter<double>("placement_x"));
        move_with_ball->setParameter("target_y", getParameter<double>("placement_y"));
      }

      skill_status = move_with_ball->run(visualizer);

      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::MOVE_TO_TARGET, SingleBallPlacementStates::PLACE_BALL,
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(
    SingleBallPlacementStates::PLACE_BALL,
    [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      if (not sleep) {
        sleep = std::make_shared<Sleep>(robot->id, world_model);
      }
      skill_status = sleep->run(visualizer);
      return Status::RUNNING;
    });

  addTransition(SingleBallPlacementStates::PLACE_BALL, SingleBallPlacementStates::SLEEP, [this]() {
    return skill_status == Status::SUCCESS;
  });

  addStateFunction(
    SingleBallPlacementStates::SLEEP,
    [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      if (not sleep) {
        sleep = std::make_shared<Sleep>(robot->id, world_model);
        sleep->setParameter("duration", 0.5);
      }
      skill_status = sleep->run(visualizer);
      return Status::RUNNING;
    });

  addTransition(SingleBallPlacementStates::SLEEP, SingleBallPlacementStates::LEAVE_BALL, [this]() {
    return skill_status == Status::SUCCESS;
  });

  addStateFunction(
    SingleBallPlacementStates::LEAVE_BALL,
    [this](ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
      if (not set_target_position) {
        set_target_position = std::make_shared<CmdSetTargetPosition>(robot->id, world_model);
      }
      // メモ：().normalized() * 0.6したらなぜかゼロベクトルが出来上がってしまう
      Vector2 diff = (robot->pose.pos - world_model->ball.pos);
      diff.normalize();
      diff = diff * 0.6;
      auto leave_pos = world_model->ball.pos + diff;
      set_target_position->setParameter("x", leave_pos.x());
      set_target_position->setParameter("y", leave_pos.y());
      set_target_position->setParameter("reach_threshold", 0.05);

      return set_target_position->run(visualizer);
    });
}
}  // namespace crane::skills
