// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BALL_PLACEMENT_PLANNER__BALL_PLACEMENT_PLANNER_HPP_
#define CRANE_BALL_PLACEMENT_PLANNER__BALL_PLACEMENT_PLANNER_HPP_

#include <boost/range/adaptor/indexed.hpp>
#include <functional>
#include <memory>

#include "crane_geometry/boost_geometry.hpp"
#include "crane_msg_wrappers/robot_command_wrapper.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/control_target.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "crane_planner_plugins/visibility_control.h"
#include <rclcpp/rclcpp.hpp>

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
class BallPlacementPlanner : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit BallPlacementPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("ball_placement_planner", options), PlannerBase("ball_placement", *this)
  {
    RCLCPP_INFO(get_logger(), "initializing");
    addRobotSelectCallback([&]() { state_ = BallPlacementState::START; });
  }

  bool isDribbleMode(const std::vector<RobotIdentifier> & robots) const
  {
    return true;
    if (robots.size() == 1) {
      return true;
    } else if (world_model_->getDistanceFromRobotToBall(robots.front()) <= 1.0) {
      return true;
    }
    return false;
  }

  bool isWallKickRequired()
  {
    return false;
    auto ball = world_model_->ball.pos;
    constexpr double OFFSET = 0.2;
    if (abs(ball.x()) > (world_model_->field_size.x() * 0.5 - OFFSET)) {
      return true;
    }
    if (abs(ball.y()) > (world_model_->field_size.y() * 0.5 - OFFSET)) {
      return true;
    }
    return false;
  }

  static constexpr double PREPARE_THRESHOLD = 0.1;
  void executeWallKickPrepare(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper> & control_targets)
  {
    Point prepare_point;  // TODO: calculate prepare_point
    crane::RobotCommandWrapper target;
    target.setID(robots.front().robot_id);
    target.kickStraight(0.5).setTargetPosition(prepare_point);
    control_targets.emplace_back(target);
    if (
      world_model_->getDistanceFromRobot({true, target.getMsg().robot_id}, prepare_point) <
      PREPARE_THRESHOLD) {
      state_ = BallPlacementState::WALL_KICK_GO;
    }
  }
  void executeWallKickGo(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper> & control_targets)
  {
    crane::RobotCommandWrapper target;
    auto robot = world_model_->getRobot(robots.front());
    target.setID(robot->id);

    auto vel = (world_model_->ball.pos - robot->pose.pos).normalized() * 0.5;
    target.kickStraight(0.5).setVelocity(vel).setTargetTheta(getAngle(vel));

    control_targets.emplace_back(target);
  }

  /**
   * @brief ボールの後ろに周りこんでドリブルの準備をする
   * @param robots
   * @param control_targets
   */
  void executePlacePrepare(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper> & control_targets)
  {
    auto target_pos =
      world_model_->ball.pos + (placement_target_ - world_model_->ball.pos).normalized() * 0.5;
    crane::RobotCommandWrapper target;
    auto robot = world_model_->getRobot(robots.front());
    target.setID(robot->id);
    target.setTargetPosition(target_pos, getAngle(world_model_->ball.pos - placement_target_));
    target.disablePlacementAvoidance();

    control_targets.emplace_back(target);

    if ((robot->pose.pos - target_pos).norm() < 0.05) {
      state_ = isDribbleMode(robots) ? BallPlacementState::PLACE_DRIBBLE_GO
                                     : BallPlacementState::PLACE_KICK_GO;
    }
  }
  void executePlaceKickGo(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper> & control_targets)
  {
  }
  void executePlaceDribbleGo(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper> & control_targets)
  {
    crane::RobotCommandWrapper target;
    auto robot = world_model_->getRobot(robots.front());
    target.setID(robot->id);
    target.dribble(0.5);
    target.disablePlacementAvoidance();

    // ball is at the back of the robot, retry the placement from preparing
    if ((placement_target_ - robot->pose.pos).dot(world_model_->ball.pos - robot->pose.pos) < 0.0) {
      state_ = BallPlacementState::PLACE_PREPARE;
    } else {
      auto vel = (robot->pose.pos - world_model_->ball.pos).normalized() * 0.2;
      target.setVelocity(vel).setTargetTheta(getAngle(vel));
      if ((world_model_->ball.pos - placement_target_).norm() < 0.03) {
        state_ = BallPlacementState::FINISH;
      }
    }
    control_targets.emplace_back(target);
  }

  void executeFinish(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane::RobotCommandWrapper> & control_targets)
  {
    crane::RobotCommandWrapper target;
    auto robot = world_model_->getRobot(robots.front());
    target.setID(robot->id);
    target.disablePlacementAvoidance();

    auto target_pos =
      world_model_->ball.pos + (robot->pose.pos - world_model_->ball.pos).normalized() * 0.5;
    target.setTargetPosition(target_pos)
      .setTargetTheta(getAngle(robot->pose.pos - world_model_->ball.pos));
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane::RobotCommandWrapper> control_targets;
    switch (state_) {
      case BallPlacementState::START: {
        state_ = (isWallKickRequired()) ? BallPlacementState::WALL_KICK_PREPARE
                                        : BallPlacementState::PLACE_PREPARE;
        // do next step
        calculateControlTarget(robots);
        break;
      }
      case BallPlacementState::WALL_KICK_PREPARE: {
        executeWallKickPrepare(robots, control_targets);
        break;
      }
      case BallPlacementState::WALL_KICK_GO: {
        executeWallKickGo(robots, control_targets);
        break;
      }
      case BallPlacementState::PLACE_PREPARE: {
        executePlacePrepare(robots, control_targets);
        break;
      }
      case BallPlacementState::PLACE_KICK_GO: {
        executePlaceKickGo(robots, control_targets);
        break;
      }
      case BallPlacementState::PLACE_DRIBBLE_GO: {
        executePlaceDribbleGo(robots, control_targets);
        break;
      }
      case BallPlacementState::FINISH: {
        executeFinish(robots, control_targets);
        break;
      }
    }

    bool required_wall_kick = false;  // TODO
    if (required_wall_kick) {
      crane_msgs::msg::RobotCommand target;
      target.robot_id = robots.front().robot_id;
      target.chip_enable = false;
      target.kick_power = 0.5;
      target.dribble_power = 0.0;
      std::vector<crane_msgs::msg::RobotCommand> cmd_msgs;
      for (const auto & cmd : control_targets) {
        cmd_msgs.push_back(cmd.getMsg());
      }
      return cmd_msgs;
    }

    for (auto r : robots | boost::adaptors::indexed()) {
      crane::RobotCommandWrapper target;
      auto robot = world_model_->getRobot(r.value());
      // common param
      target.setID(robot->id);
      target.disablePlacementAvoidance();

      bool dribble_mode = true;  // TODO
      if (dribble_mode) {
        if (r.index() > 0) {
          target.getEditableMsg().disable_placement_avoidance = false;
        } else {
        }
      }

      // TODO: implement
      target.setTargetPosition(0.0, 0.0, 0.0);

      control_targets.emplace_back(target);
    }
    std::vector<crane_msgs::msg::RobotCommand> cmd_msgs;
    for (const auto & cmd : control_targets) {
      cmd_msgs.push_back(cmd.getMsg());
    }
    return cmd_msgs;
  }
  double getRoleScore(std::shared_ptr<RobotInfo> robot) override
  {
    // the nearest to the ball first
    return 100. / world_model_->getSquareDistanceFromRobotToBall({true, robot->id});
  }

private:
  BallPlacementState state_;
  Point placement_target_;
};

}  // namespace crane

#endif  // CRANE_BALL_PLACEMENT_PLANNER__BALL_PLACEMENT_PLANNER_HPP_
