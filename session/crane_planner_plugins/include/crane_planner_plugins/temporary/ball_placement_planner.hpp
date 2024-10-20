// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__TEMPORARY__BALL_PLACEMENT_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__TEMPORARY__BALL_PLACEMENT_PLANNER_HPP_

#include <algorithm>
#include <boost/range/adaptor/indexed.hpp>
#include <crane_basics/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../visibility_control.h"

namespace crane
{
enum class BallPlacementState {
  START,
  WALL_KICK_PREPARE,
  WALL_KICK_GO,
  PLACE_PREPARE,
  PLACE_KICK_GO,
  PLACE_DRIBBLE_GO,
  FINISH,
};
class BallPlacementPlanner : public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit BallPlacementPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("ball_placement", world_model, visualizer)
  {
    addRobotSelectCallback([&]() { state = BallPlacementState::START; });
  }

  bool isDribbleMode(const std::vector<RobotIdentifier> & robots) const
  {
    return true;
    if (robots.size() == 1) {
      return true;
    } else if (world_model->getDistanceFromRobotToBall(robots.front()) <= 1.0) {
      return true;
    }
    return false;
  }

  bool isWallKickRequired()
  {
    return false;
    auto ball = world_model->ball.pos;
    constexpr double OFFSET = 0.2;
    if (abs(ball.x()) > (world_model->field_size.x() * 0.5 - OFFSET)) {
      return true;
    }
    if (abs(ball.y()) > (world_model->field_size.y() * 0.5 - OFFSET)) {
      return true;
    }
    return false;
  }

  static constexpr double PREPARE_THRESHOLD = 0.1;
  void executeWallKickPrepare(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper::SharedPtr> & robot_commands)
  {
    Point prepare_point;  // TODO(HansRobo): calculate prepare_point
    auto command = std::make_shared<crane::RobotCommandWrapper>(
      "ball_placement_planner", robots.front().robot_id, world_model);
    command->kickStraight(0.5).setTargetPosition(prepare_point);
    robot_commands.emplace_back(command);
    if (
      world_model->getDistanceFromRobot(command->getMsg().robot_id, prepare_point) <
      PREPARE_THRESHOLD) {
      state = BallPlacementState::WALL_KICK_GO;
    }
  }

  void executeWallKickGo(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper::SharedPtr> & robot_commands)
  {
    auto command = std::make_shared<crane::RobotCommandWrapper>(
      "ball_placement_planner", robots.front().robot_id, world_model);
    auto robot = world_model->getRobot(robots.front());

    auto vel = (world_model->ball.pos - robot->pose.pos).normalized() * 0.5;
    command->kickStraight(0.5).setVelocity(vel).setTargetTheta(getAngle(vel));

    robot_commands.emplace_back(command);
  }

  /**
   * @brief ボールの後ろに周りこんでドリブルの準備をする
   * @param robots
   * @param robot_commands
   */
  void executePlacePrepare(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper::SharedPtr> & robot_commands)
  {
    auto target_pos =
      world_model->ball.pos + (placement_target - world_model->ball.pos).normalized() * 0.5;
    auto command = std::make_shared<crane::RobotCommandWrapper>(
      "ball_placement_planner", robots.front().robot_id, world_model);
    auto robot = world_model->getRobot(robots.front());
    command->setTargetPosition(target_pos)
      .setTargetAngle(getAngle(world_model->ball.pos - placement_target));
    command->disablePlacementAvoidance();

    robot_commands.emplace_back(command);

    if ((robot->pose.pos - target_pos).norm() < 0.05) {
      state = isDribbleMode(robots) ? BallPlacementState::PLACE_DRIBBLE_GO
                                    : BallPlacementState::PLACE_KICK_GO;
    }
  }

  void executePlaceKickGo(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper::SharedPtr> & robot_commands)
  {
  }

  void executePlaceDribbleGo(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper::SharedPtr> & robot_commands)
  {
    auto command = std::make_shared<crane::RobotCommandWrapper>(
      "ball_placement_planner", robots.front().robot_id, world_model);
    auto robot = world_model->getRobot(robots.front());
    command->dribble(0.5);
    command->disablePlacementAvoidance();

    // ball is at the back of the robot, retry the placement from preparing
    if ((placement_target - robot->pose.pos).dot(world_model->ball.pos - robot->pose.pos) < 0.0) {
      state = BallPlacementState::PLACE_PREPARE;
    } else {
      auto vel = (robot->pose.pos - world_model->ball.pos).normalized() * 0.2;
      command->setVelocity(vel).setTargetTheta(getAngle(vel));
      if ((world_model->ball.pos - placement_target).norm() < 0.03) {
        state = BallPlacementState::FINISH;
      }
    }
    robot_commands.emplace_back(command);
  }

  void executeFinish(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper::SharedPtr> & robot_commands)
  {
    auto command = std::make_shared<crane::RobotCommandWrapper>(
      "ball_placement_planner", robots.front().robot_id, world_model);
    auto robot = world_model->getRobot(robots.front());
    command->disablePlacementAvoidance();
    auto target_pos =
      world_model->ball.pos + (robot->pose.pos - world_model->ball.pos).normalized() * 0.5;
    command->setTargetPosition(target_pos)
      .setTargetTheta(getAngle(robot->pose.pos - world_model->ball.pos));
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane::RobotCommandWrapper::SharedPtr> robot_commands;
    switch (state) {
      case BallPlacementState::START: {
        state = (isWallKickRequired()) ? BallPlacementState::WALL_KICK_PREPARE
                                       : BallPlacementState::PLACE_PREPARE;
        // do next step
        calculateRobotCommand(robots);
        break;
      }
      case BallPlacementState::WALL_KICK_PREPARE: {
        executeWallKickPrepare(robots, robot_commands);
        break;
      }
      case BallPlacementState::WALL_KICK_GO: {
        executeWallKickGo(robots, robot_commands);
        break;
      }
      case BallPlacementState::PLACE_PREPARE: {
        executePlacePrepare(robots, robot_commands);
        break;
      }
      case BallPlacementState::PLACE_KICK_GO: {
        executePlaceKickGo(robots, robot_commands);
        break;
      }
      case BallPlacementState::PLACE_DRIBBLE_GO: {
        executePlaceDribbleGo(robots, robot_commands);
        break;
      }
      case BallPlacementState::FINISH: {
        executeFinish(robots, robot_commands);
        break;
      }
    }

    bool required_wall_kick = false;  // TODO(HansRobo): implement
    if (required_wall_kick) {
      crane_msgs::msg::RobotCommand target;
      target.robot_id = robots.front().robot_id;
      target.chip_enable = false;
      target.kick_power = 0.5;
      target.dribble_power = 0.0;

      std::vector<crane_msgs::msg::RobotCommand> cmd_msgs;
      for (const auto & cmd : robot_commands) {
        cmd_msgs.push_back(cmd->getMsg());
      }
      return {PlannerBase::Status::RUNNING, cmd_msgs};
      //      return cmd_msgs;
    }

    for (auto r : robots | boost::adaptors::indexed()) {
      auto command = std::make_shared<crane::RobotCommandWrapper>(
        "ball_placement_planner", r.value().robot_id, world_model);
      // common param
      command->disablePlacementAvoidance();

      bool dribble_mode = true;  // TODO(HansRobo): implement
      if (dribble_mode) {
        if (r.index() > 0) {
          command->enablePlacementAvoidance();
        } else {
        }
      }

      // TODO(HansRobo): implement
      command->setTargetPosition(0.0, 0.0).setTargetTheta(0.0);

      robot_commands.emplace_back(command);
    }
    std::vector<crane_msgs::msg::RobotCommand> cmd_msgs;
    for (const auto & cmd : robot_commands) {
      cmd_msgs.push_back(cmd->getMsg());
    }
    return {PlannerBase::Status::RUNNING, cmd_msgs};
  }

  auto getSelectedRobots(
    uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        // ボールに近いほどスコアが高い
        return 100.0 / std::max(world_model->getSquareDistanceFromRobotToBall(robot->id), 0.01);
      },
      prev_roles);
  }

private:
  BallPlacementState state;

  Point placement_target;
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__TEMPORARY__BALL_PLACEMENT_PLANNER_HPP_
