// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__TIGERS_GOALIE_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__TIGERS_GOALIE_PLANNER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/control_target.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "visibility_control.h"

namespace crane
{
class TigersGoaliePlanner : public PlannerBase
{
public:
  enum class State {
    STOP,
    DEFEND,
    INTERCEPT,
    PASS,
    MOVE_TO_PENALTY_AREA,
    RAMBO,
    MOVE_IN_FRONT_OF_BALL,
    GET_BALL_CONTACT,
    MOVE_WITH_BALL,
    PREPARE_PENALTY,
  };

  enum class Status {
    SUCCESS,
    FAILURE,
    RUNNING,
  };
  COMPOSITION_PUBLIC
  explicit TigersGoaliePlanner(
    WorldModelWrapper::SharedPtr & world_model, ConsaiVisualizerWrapper::SharedPtr visualizer)
  : PlannerBase("tigers_goalie", world_model, visualizer)
  {
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    auto robot = world_model->getRobot(robots.front());
    crane::RobotCommandWrapper command(robot->id, world_model);
    switch (state) {
      case State::STOP:
        // KeeperStoppedState
        if (not isStopped()) {
          state = State::DEFEND;
        }
        break;
      case State::PREPARE_PENALTY:
        // PreparePenaltyState
        if (not isPreparePenalty()) {
          state = State::DEFEND;
        }
        break;
      case State::MOVE_TO_PENALTY_AREA: {
        // MoveToPenaltyAreaState
        Status status;
        if (status == Status::SUCCESS) {
          state = State::DEFEND;
        } else if (isKeeperWellInsidePenaltyArea()) {
          state = State::DEFEND;
        } else if (isStopped()) {
          state = State::STOP;
        } else if (isPreparePenalty()) {
          state = State::PREPARE_PENALTY;
        }
        break;
      }
      case State::DEFEND: {
        auto status = doCriticalKeeper(robot, command);
        if (ballCanBePassedOutOfPenaltyArea()) {
          state = State::PASS;
        } else if (canGoOut()) {
          state = State::RAMBO;
        } else if (isBallBetweenGoalieAndGoal()) {
          state = State::GET_BALL_CONTACT;
        } else if (isOutsidePenaltyArea()) {
          state = State::MOVE_TO_PENALTY_AREA;
        } else if (isStopped()) {
          state = State::STOP;
        } else if (isPreparePenalty()) {
          state = State::PREPARE_PENALTY;
        } else if (canInterceptSafely()) {
          state = State::INTERCEPT;
        }
        break;
      }
      case State::PASS: {
        // PassState
        if (isBallMoving()) {
          state = State::DEFEND;
        } else if (isBallPlacementRequired()) {
          state = State::MOVE_IN_FRONT_OF_BALL;
        } else if (isStopped()) {
          state = State::STOP;
        } else if (isPreparePenalty()) {
          state = State::PREPARE_PENALTY;
        }
        break;
      }
      case State::INTERCEPT: {
        // InterceptRollingBallState
        if (hasInterceptionFailed(robot)) {
          state = State::DEFEND;
        } else if (ballCanBePassedOutOfPenaltyArea()) {
          state = State::PASS;
        } else if (isStopped()) {
          state = State::STOP;
        } else if (isPreparePenalty()) {
          state = State::PREPARE_PENALTY;
        }
        break;
      }
      case State::RAMBO: {
        // RamboKeeper
        if (world_model->isDefenseArea(world_model->ball.pos) or isGoalKick()) {
          state = State::DEFEND;
        } else if (isStopped()) {
          state = State::STOP;
        } else if (isPreparePenalty()) {
          state = State::PREPARE_PENALTY;
        }
        break;
      }
      case State::MOVE_IN_FRONT_OF_BALL: {
        // MoveInFrontOfBallState
        Status status;
        if (isBallMoving()) {
          state = State::DEFEND;
        } else if (isBallPlaced()) {
          state = State::DEFEND;
        } else if (status == Status::SUCCESS) {
          state = State::GET_BALL_CONTACT;
        } else if (isStopped()) {
          state = State::STOP;
        } else if (isPreparePenalty()) {
          state = State::PREPARE_PENALTY;
        }
        break;
      }
      case State::GET_BALL_CONTACT: {
        // doGetBallContact
        Status status;
        if (status == Status::SUCCESS) {
          state = State::MOVE_WITH_BALL;
        } else if (status == Status::FAILURE) {
          state = State::MOVE_IN_FRONT_OF_BALL;
        } else if (isStopped()) {
          state = State::STOP;
        } else if (isPreparePenalty()) {
          state = State::PREPARE_PENALTY;
        }
        break;
      }
      case State::MOVE_WITH_BALL: {
        // MoveWithBallState
        Status status;
        if (status == Status::SUCCESS) {
          state = State::DEFEND;
        } else if (status == Status::FAILURE) {
          state = State::MOVE_IN_FRONT_OF_BALL;
        } else if (isStopped()) {
          state = State::STOP;
        } else if (isPreparePenalty()) {
          state = State::PREPARE_PENALTY;
        }
        break;
      }
    }
  }

  Status doCriticalKeeper(const std::shared_ptr<RobotInfo> & robot, RobotCommandWrapper & command)
  {
    return Status::SUCCESS;
  }

  bool isBallMoveToweredTo(Point point)
  {
    double dot =
      (point - world_model->ball.pos).normalized().dot(world_model->ball.vel.normalized());
    return dot > 0.5 or (point - world_model->ball.pos).norm() < 0.2;
    ;
  }

  bool isBallAimedForGoal()
  {
    Segment goal_line{world_model->getOurGoalPosts().first, world_model->getOurGoalPosts().second};
    Segment ball_line{
      world_model->ball.pos, world_model->ball.pos + world_model->ball.vel.normalized() * 10.0};
    return boost::geometry::intersects(goal_line, ball_line);
  }

  bool isStopped() const { return false; }

  bool isPreparePenalty() const { return false; }

  bool isKeeperWellInsidePenaltyArea() const { return false; }

  bool ballCanBePassedOutOfPenaltyArea() const { return false; }

  bool canGoOut() const { return false; }

  bool isBallBetweenGoalieAndGoal() const { return false; }

  bool isOutsidePenaltyArea() const { return false; }

  bool canInterceptSafely()
  {
    return false;
    //    return world_model->isDefenseArea(world_model->ball.pos) && (not isBallAimedForGoal());
  }

  bool isBallMoving() const { return false; }

  bool isBallPlacementRequired() const { return false; }
  bool hasInterceptionFailed(const std::shared_ptr<RobotInfo> & robot)
  {
    return isBallMoveToweredTo(robot->pose.pos) or
           not world_model->isDefenseArea(world_model->ball.pos);
  }

  bool isGoalKick() const { return false; }

  bool isBallPlaced() const { return false; }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots)
    -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots, [this](const std::shared_ptr<RobotInfo> & robot) {
        // choose id smaller first
        return 15. - static_cast<double>(-robot->id);
      });
  }

  State state = State::DEFEND;
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__TIGERS_GOALIE_PLANNER_HPP_
