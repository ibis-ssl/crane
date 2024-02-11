// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__BALL_PLACEMENT_WITH_SKILL_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__BALL_PLACEMENT_WITH_SKILL_PLANNER_HPP_

#include <boost/range/adaptor/indexed.hpp>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/control_target.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/get_ball_contact.hpp>
#include <crane_robot_skills/move_with_ball.hpp>
#include <crane_robot_skills/turn_around_point.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{

class BallPlacementWithSkillPlanner : public PlannerBase
{
public:
  enum class BallPlacementState {
    GO_TO_BALL,
    TURN,
    GET_BALL_CONTACT,
    MOVE_WITH_BALL,
    CLEAR_BALL,
  };

  COMPOSITION_PUBLIC
  explicit BallPlacementWithSkillPlanner(
    WorldModelWrapper::SharedPtr & world_model, ConsaiVisualizerWrapper::SharedPtr visualizer)
  : PlannerBase("ball_placement_with_skill", world_model, visualizer),
    state(BallPlacementState::GO_TO_BALL)
  {
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    if (robots.size() != 1) {
      return {};
    }

    placement_target = world_model->getBallPlacementTarget().value();

    auto robot = world_model->getRobot(robots.front());

    if (not move_with_ball) {
      move_with_ball = std::make_unique<MoveWithBall>(robot->id, world_model);
      move_with_ball->setTargetPose([&]() {
        Pose2D pose;
        pose.pos = placement_target + (world_model->ball.pos - placement_target).normalized() *
                                        robot->center_to_kicker().norm();
        pose.theta = getAngle(placement_target - world_model->ball.pos);
        return pose;
      }());
    }

    if (not get_ball_contact) {
      get_ball_contact = std::make_unique<GetBallContact>(robot->id, world_model);
    }

    crane::RobotCommandWrapper command(robot->id, world_model);

    switch (state) {
      case BallPlacementState::GO_TO_BALL: {
        command.setTargetPosition(
          world_model->ball.pos + (robot->pose.pos - world_model->ball.pos).normalized() * 0.15);
        command.setTargetTheta(getAngle(world_model->ball.pos - robot->pose.pos));
        command.setMaxVelocity(2.0);
        command.setTerminalVelocity(0.2);
        if (auto distance = world_model->getDistanceFromRobotToBall(robot->id);
            distance < 0.20 && distance > 0.15) {
          state = BallPlacementState::TURN;
        }
        break;
      }

      case BallPlacementState::TURN: {
        if (not turn_around_point) {
          turn_around_point = std::make_unique<TurnAroundPoint>(robot->id, world_model);
          turn_around_point->setTargetPoint(world_model->ball.pos);
          turn_around_point->setTargetAngle(getAngle(world_model->ball.pos - placement_target));
          turn_around_point->setParameter("max_velocity", 1.5);
          turn_around_point->setParameter("max_turn_omega", M_PI);
        }
        if (turn_around_point->run(command, visualizer) == SkillBase<>::Status::SUCCESS) {
          std::cout << "GET_BALL_CONTACT" << std::endl;
          state = BallPlacementState::GET_BALL_CONTACT;
          turn_around_point = nullptr;
        }
        command.setMaxVelocity(1.0);
        command.setTerminalVelocity(0.5);
        break;
      }

      case BallPlacementState::GET_BALL_CONTACT: {
        if (get_ball_contact->run(command, visualizer) == SkillBase<>::Status::SUCCESS) {
          Pose2D target_pose;
          target_pose.pos =
            placement_target + (world_model->ball.pos - placement_target).normalized() *
                                 robot->center_to_kicker().norm();
          target_pose.theta = getAngle(placement_target - world_model->ball.pos);
          move_with_ball->setTargetPose(target_pose);
          move_with_ball_success_count = 0;
          state = BallPlacementState::MOVE_WITH_BALL;
        }
        command.setMaxVelocity(0.5);
        break;
      }

      case BallPlacementState::MOVE_WITH_BALL: {
        auto status = move_with_ball->run(command, visualizer);
        command.setMaxVelocity(0.5);
        command.setTerminalVelocity(0.1);
        //      command.setTerminalVelocity(
        //        std::min(1.0, std::max((double)(robot->pose.pos - placement_target).norm() - 0.1, 0.0)));
        command.setMaxOmega(M_PI / 2.0);
        if (status == SkillBase<>::Status::FAILURE) {
          state = BallPlacementState::GO_TO_BALL;
        } else if (status == SkillBase<>::Status::SUCCESS) {
          move_with_ball_success_count++;
          if (move_with_ball_success_count >= 20) {
            state = BallPlacementState::CLEAR_BALL;
          }
        } else {
          move_with_ball_success_count = 0;
        }
        break;
      }

      case BallPlacementState::CLEAR_BALL: {
        command.setTargetPosition(
          placement_target + (robot->pose.pos - placement_target).normalized() * 0.6);
        command.setMaxVelocity(0.5);
        break;
      }
    }

    std::vector<crane_msgs::msg::RobotCommand> cmd_msgs{command.getMsg()};

    return cmd_msgs;
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
        // ボールに近いほどスコアが高い
        return 100.0 /
               std::max(world_model->getSquareDistanceFromRobotToBall({true, robot->id}), 0.01);
      });
  }

private:
  BallPlacementState state;

  std::unique_ptr<GetBallContact> get_ball_contact = nullptr;

  std::unique_ptr<MoveWithBall> move_with_ball = nullptr;

  std::unique_ptr<TurnAroundPoint> turn_around_point = nullptr;

  Point placement_target = Point(0.0, 0.0);

  int move_with_ball_success_count = 0;
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__BALL_PLACEMENT_WITH_SKILL_PLANNER_HPP_
