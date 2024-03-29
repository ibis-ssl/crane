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
    "SingleBallPlacement", id, wm, SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE)
{
  setParameter("placement_x", 0.);
  setParameter("placement_y", 0.);

  // マイナスするとコート内も判定される
  setParameter("コート端判定のオフセット", -0.5);

  addStateFunction(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not pull_back_target) {
        pull_back_target = world_model->ball.pos;
        const auto offset = getParameter<double>("コート端判定のオフセット");
        const auto threshold_x = world_model->field_size.x() * 0.5 + offset;
        const auto threshold_y = world_model->field_size.y() * 0.5 + offset;
        if (std::abs(pull_back_target->x()) > threshold_x) {
          pull_back_target->x() = std::copysign(threshold_x, pull_back_target->x());
        }
        if (std::abs(pull_back_target->y()) > threshold_y) {
          pull_back_target->y() = std::copysign(threshold_y, pull_back_target->y());
        }
      }
      command->setTargetPosition(pull_back_target.value());
      command->lookAtBallFrom(pull_back_target.value());
      command->disablePlacementAvoidance();
      return Status::RUNNING;
    });

  // 必要ない場合はコート端処理をスキップ
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE, SingleBallPlacementStates::GO_OVER_BALL,
    [this]() {
      return world_model->isFieldInside(
        world_model->ball.pos, getParameter<double>("コート端判定のオフセット"));
    });

  // pull_back_targetに到達したら次のステートへ
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH, [this]() {
      if (not pull_back_target) {
        return false;
      } else {
        skill_status = Status::RUNNING;
        return command->robot->getDistance(pull_back_target.value()) < 0.05;
      }
    });

  // PULL_BACK_FROM_EDGE_TOUCH
  addStateFunction(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not get_ball_contact) {
        get_ball_contact = std::make_shared<GetBallContact>(robot->id, world_model);
        get_ball_contact->setCommander(command);
        get_ball_contact->setParameter("min_contact_duration", 2.0);
      }
      skill_status = get_ball_contact->run(visualizer);
      command->dribble(0.5);
      command->disablePlacementAvoidance();

      return skill_status;
    });

  // skill_status == Status::SUCCESSの場合に次のステートへ
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_TOUCH,
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL, [this]() {
      if (not get_ball_contact) {
        return false;
      } else {
        return skill_status == Status::SUCCESS;
      }
    });

  // PULL_BACK_FROM_EDGE_PULL
  addStateFunction(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      command->setTargetPosition(pull_back_target.value());
      command->dribble(0.5);
      command->setMaxVelocity(0.2);
      command->disablePlacementAvoidance();
      return Status::RUNNING;
    });

  // pull_back_targetに到着したら次のステートへ
  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PULL, SingleBallPlacementStates::GO_OVER_BALL,
    [this]() { return command->robot->getDistance(pull_back_target.value()) < 0.05; });

  addTransition(
    SingleBallPlacementStates::PULL_BACK_FROM_EDGE_PREPARE, SingleBallPlacementStates::GO_OVER_BALL,
    [this]() {
      return world_model->isFieldInside(
        world_model->ball.pos, getParameter<double>("コート端判定のオフセット"));
    });

  addStateFunction(
    SingleBallPlacementStates::GO_OVER_BALL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not go_over_ball) {
        command->dribble(0.0);
        go_over_ball = std::make_shared<GoOverBall>(robot->id, world_model);
        go_over_ball->setCommander(command);
        go_over_ball->setParameter("next_target_x", getParameter<double>("placement_x"));
        go_over_ball->setParameter("next_target_y", getParameter<double>("placement_y"));
        go_over_ball->setParameter("margin", 0.3);
      }
      command->setMaxVelocity(1.0);
      command->disablePlacementAvoidance();

      skill_status = go_over_ball->run(visualizer);

      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::GO_OVER_BALL, SingleBallPlacementStates::CONTACT_BALL,
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(
    SingleBallPlacementStates::CONTACT_BALL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not get_ball_contact) {
        get_ball_contact = std::make_shared<GetBallContact>(robot->id, world_model);
        get_ball_contact->setCommander(command);
      }

      skill_status = get_ball_contact->run(visualizer);
      command->disablePlacementAvoidance();

      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::CONTACT_BALL, SingleBallPlacementStates::MOVE_TO_TARGET,
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(
    SingleBallPlacementStates::MOVE_TO_TARGET,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not move_with_ball) {
        move_with_ball = std::make_shared<MoveWithBall>(robot->id, world_model);
        move_with_ball->setCommander(command);
        move_with_ball->setParameter("target_x", getParameter<double>("placement_x"));
        move_with_ball->setParameter("target_y", getParameter<double>("placement_y"));
      }

      skill_status = move_with_ball->run(visualizer);
      command->disablePlacementAvoidance();
      return Status::RUNNING;
    });

  addTransition(
    SingleBallPlacementStates::MOVE_TO_TARGET, SingleBallPlacementStates::PLACE_BALL,
    [this]() { return skill_status == Status::SUCCESS; });

  addStateFunction(
    SingleBallPlacementStates::PLACE_BALL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not sleep) {
        sleep = std::make_shared<Sleep>(robot->id, world_model);
        sleep->setCommander(command);
      }
      skill_status = sleep->run(visualizer);
      return Status::RUNNING;
    });

  addTransition(SingleBallPlacementStates::PLACE_BALL, SingleBallPlacementStates::SLEEP, [this]() {
    return skill_status == Status::SUCCESS;
  });

  addStateFunction(
    SingleBallPlacementStates::SLEEP,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not sleep) {
        sleep = std::make_shared<Sleep>(robot->id, world_model);
        sleep->setCommander(command);
        sleep->setParameter("duration", 0.5);
      }
      skill_status = sleep->run(visualizer);
      command->disablePlacementAvoidance();
      return Status::RUNNING;
    });

  addTransition(SingleBallPlacementStates::SLEEP, SingleBallPlacementStates::LEAVE_BALL, [this]() {
    return skill_status == Status::SUCCESS;
  });

  addStateFunction(
    SingleBallPlacementStates::LEAVE_BALL,
    [this](const ConsaiVisualizerWrapper::SharedPtr & visualizer) -> Status {
      if (not set_target_position) {
        set_target_position = std::make_shared<CmdSetTargetPosition>(robot->id, world_model);
        set_target_position->setCommander(command);
      }
      // メモ：().normalized() * 0.6したらなぜかゼロベクトルが出来上がってしまう
      Vector2 diff = (robot->pose.pos - world_model->ball.pos);
      diff.normalize();
      diff = diff * 0.6;
      auto leave_pos = world_model->ball.pos + diff;
      set_target_position->setParameter("x", leave_pos.x());
      set_target_position->setParameter("y", leave_pos.y());
      set_target_position->setParameter("reach_threshold", 0.05);

      command->disablePlacementAvoidance();
      return set_target_position->run(visualizer);
    });
}
}  // namespace crane::skills
