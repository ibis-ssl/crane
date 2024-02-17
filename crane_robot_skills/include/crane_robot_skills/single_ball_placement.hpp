// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
#define CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_

#include <crane_geometry/eigen_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>

#include "get_ball_contact.hpp"
#include "go_over_ball.hpp"
#include "move_with_ball.hpp"
#include "robot_command_as_skill.hpp"
#include "sleep.hpp"

namespace crane::skills
{
enum class SingleBallPlacementStates {
  GO_OVER_BALL,
  CONTACT_BALL,
  MOVE_TO_TARGET,
  PLACE_BALL,
  SLEEP,
  LEAVE_BALL,
};

class SingleBallPlacement : public SkillBase<SingleBallPlacementStates>
{
private:
  std::shared_ptr<GoOverBall> go_over_ball;

  std::shared_ptr<GetBallContact> get_ball_contact;

  std::shared_ptr<MoveWithBall> move_with_ball;

  std::shared_ptr<Sleep> sleep = nullptr;

  std::shared_ptr<CmdSetTargetPosition> set_target_position;

  Status skill_status = Status::RUNNING;

public:
  explicit SingleBallPlacement(uint8_t id, const std::shared_ptr<WorldModelWrapper> & world_model)
  : SkillBase<SingleBallPlacementStates>(
      "SingleBallPlacement", id, world_model, SingleBallPlacementStates::GO_OVER_BALL)
  {
    setParameter("placement_x", 0.);
    setParameter("placement_y", 0.);
    addStateFunction(
      SingleBallPlacementStates::GO_OVER_BALL,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        if (not go_over_ball) {
          go_over_ball = std::make_shared<GoOverBall>(robot->id, world_model);
          go_over_ball->setParameter("next_target_x", getParameter<double>("placement_x"));
          go_over_ball->setParameter("next_target_y", getParameter<double>("placement_y"));
          go_over_ball->setParameter("margin", 0.4);
        }

        skill_status = go_over_ball->run(command, visualizer);

        return Status::RUNNING;
      });

    addTransition(
      SingleBallPlacementStates::GO_OVER_BALL, SingleBallPlacementStates::CONTACT_BALL,
      [this]() { return skill_status == Status::SUCCESS; });

    addStateFunction(
      SingleBallPlacementStates::CONTACT_BALL,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        if (not get_ball_contact) {
          get_ball_contact = std::make_shared<GetBallContact>(robot->id, world_model);
        }

        skill_status = get_ball_contact->run(command, visualizer);

        return Status::RUNNING;
      });

    addTransition(
      SingleBallPlacementStates::CONTACT_BALL, SingleBallPlacementStates::MOVE_TO_TARGET,
      [this]() { return skill_status == Status::SUCCESS; });

    addStateFunction(
      SingleBallPlacementStates::MOVE_TO_TARGET,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        if (not move_with_ball) {
          move_with_ball = std::make_shared<MoveWithBall>(robot->id, world_model);
          move_with_ball->setParameter("target_x", getParameter<double>("placement_x"));
          move_with_ball->setParameter("target_y", getParameter<double>("placement_y"));
          double target_theta = getAngle(
            Point(getParameter<double>("placement_x"), getParameter<double>("placement_y")) -
            world_model->ball.pos);
          move_with_ball->setParameter("target_theta", target_theta);
        }

        skill_status = move_with_ball->run(command, visualizer);

        return Status::RUNNING;
      });

    addTransition(
      SingleBallPlacementStates::MOVE_TO_TARGET, SingleBallPlacementStates::PLACE_BALL,
      [this]() { return skill_status == Status::SUCCESS; });

    addStateFunction(
      SingleBallPlacementStates::PLACE_BALL,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        if (not sleep) {
          sleep = std::make_shared<Sleep>(robot->id, world_model);
        }
        skill_status = sleep->run(command, visualizer);
        return Status::RUNNING;
      });

    addTransition(
      SingleBallPlacementStates::PLACE_BALL, SingleBallPlacementStates::SLEEP,
      [this]() { return skill_status == Status::SUCCESS; });

    addStateFunction(
      SingleBallPlacementStates::SLEEP,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        if (not sleep) {
          sleep = std::make_shared<Sleep>(robot->id, world_model);
          sleep->setParameter("duration", 0.5);
        }
        skill_status = sleep->run(command, visualizer);
        return Status::RUNNING;
      });

    addTransition(
      SingleBallPlacementStates::SLEEP, SingleBallPlacementStates::LEAVE_BALL,
      [this]() { return skill_status == Status::SUCCESS; });

    addStateFunction(
      SingleBallPlacementStates::LEAVE_BALL,
      [this](
        const std::shared_ptr<WorldModelWrapper> & world_model,
        const std::shared_ptr<RobotInfo> & robot, crane::RobotCommandWrapper & command,
        ConsaiVisualizerWrapper::SharedPtr visualizer) -> Status {
        if (not set_target_position) {
          set_target_position =
            std::make_shared<CmdSetTargetPosition>(robot->id, world_model);
          auto leave_pos =
            world_model->ball.pos - getNormVec(robot->pose.theta) * 0.6;
          set_target_position->setParameter("x", leave_pos.x());
          set_target_position->setParameter("y", leave_pos.y());
          set_target_position->setParameter("reach_threshold", 0.05);
        }

        return set_target_position->run(command, visualizer);
      });
  }

  void print(std::ostream & os) const override
  {
    os << "[SingleBallPlacement]";

    switch (getCurrentState()) {
      case SingleBallPlacementStates::GO_OVER_BALL:
        go_over_ball->print(os);
        break;
      case SingleBallPlacementStates::CONTACT_BALL:
        get_ball_contact->print(os);
        break;
      case SingleBallPlacementStates::MOVE_TO_TARGET:
        move_with_ball->print(os);
        break;
      case SingleBallPlacementStates::PLACE_BALL:
        os << " PLACE_BALL";
        break;
      case SingleBallPlacementStates::SLEEP:
        sleep->print(os);
        break;
      case SingleBallPlacementStates::LEAVE_BALL:
        set_target_position->print(os);
        break;
      default:
        os << " UNKNOWN";
        break;
    }
  }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
