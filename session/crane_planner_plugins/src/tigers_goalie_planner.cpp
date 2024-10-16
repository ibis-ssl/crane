// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_planner_plugins/tigers_goalie_planner.hpp>

namespace crane
{
std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
TigersGoaliePlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  auto robot = world_model->getRobot(robots.front());
  crane::RobotCommandWrapperPosition command("tigers_goalie_planner", robot->id, world_model);
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
      Status status = Status::RUNNING;
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
      [[maybe_unused]] auto status = doCriticalKeeper(robot, command);
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
      if (world_model->point_checker.isPenaltyArea(world_model->ball.pos) or isGoalKick()) {
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
      Status status = Status::RUNNING;
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
      Status status = Status::RUNNING;
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
      Status status = Status::RUNNING;
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
  return {PlannerBase::Status::RUNNING, {}};
}

auto TigersGoaliePlanner::getSelectedRobots(
  uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t>
{
  return this->getSelectedRobotsByScore(
    selectable_robots_num, selectable_robots,
    [this](const std::shared_ptr<RobotInfo> & robot) {
      // choose id smaller first
      return 15. - static_cast<double>(-robot->id);
    },
    prev_roles);
}
}  // namespace crane
