// Copyright (c) 2022 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_BALL_PLACEMENT_PLANNER__BALL_PLACEMENT_PLANNER_HPP_
#define CRANE_BALL_PLACEMENT_PLANNER__BALL_PLACEMENT_PLANNER_HPP_

#include <boost/range/adaptor/indexed.hpp>
#include <functional>
#include <memory>

#include "crane_ball_placement_planner/visibility_control.h"
#include "crane_geometry/boost_geometry.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/control_target.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_planner_base/planner_base.hpp"
#include "rclcpp/rclcpp.hpp"

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
class BallPlacementPlannerComponent : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit BallPlacementPlannerComponent(const rclcpp::NodeOptions & options)
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
    std::vector<crane_msgs::msg::RobotCommand> & control_targets)
  {
    Point prepare_point;  // TODO: calculate prepare_point
    crane_msgs::msg::RobotCommand target;
    target.robot_id = robots.front().robot_id;
    target.chip_enable = false;
    target.kick_power = 0.5;
    target.dribble_power = 0.0;
    target.motion_mode_enable = false;
    target.target.x = prepare_point.x();
    target.target.y = prepare_point.y();
    control_targets.emplace_back(target);
    if (
      world_model_->getDistanceFromRobot({true, target.robot_id}, prepare_point) <
      PREPARE_THRESHOLD) {
      state_ = BallPlacementState::WALL_KICK_GO;
    }
  }
  void executeWallKickGo(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane_msgs::msg::RobotCommand> & control_targets)
  {
    crane_msgs::msg::RobotCommand target;
    auto robot = world_model_->getRobot(robots.front());
    target.robot_id = robot->id;
    target.chip_enable = false;
    target.kick_power = 0.5;
    target.dribble_power = 0.0;
    target.motion_mode_enable = true;

    auto vel = (world_model_->ball.pos - robot->pose.pos).normalized() * 0.5;
    target.target.x = vel.x();
    target.target.y = vel.y();
    target.target.theta = getAngle(vel);
    control_targets.emplace_back(target);
  }

  /**
   * @brief ボールの後ろに周りこんでドリブルの準備をする
   * @param robots
   * @param control_targets
   */
  void executePlacePrepare(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane_msgs::msg::RobotCommand> & control_targets)
  {
    auto target_pos =
      world_model_->ball.pos + (placement_target_ - world_model_->ball.pos).normalized() * 0.5;
    crane_msgs::msg::RobotCommand target;
    auto robot = world_model_->getRobot(robots.front());
    target.robot_id = robot->id;
    target.chip_enable = false;
    target.motion_mode_enable = false;
    target.disable_placement_avoidance = true;

    target.target.x = target_pos.x();
    target.target.y = target_pos.y();
    target.target.theta = getAngle(world_model_->ball.pos - placement_target_);
    control_targets.emplace_back(target);

    if ((robot->pose.pos - target_pos).norm() < 0.05) {
      state_ = isDribbleMode(robots) ? BallPlacementState::PLACE_DRIBBLE_GO
                                     : BallPlacementState::PLACE_KICK_GO;
    }
  }
  void executePlaceKickGo(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane_msgs::msg::RobotCommand> & control_targets)
  {
  }
  void executePlaceDribbleGo(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane_msgs::msg::RobotCommand> & control_targets)
  {
  }
  void executeFinish(
    const std::vector<RobotIdentifier> & robots,
    std::vector<crane_msgs::msg::RobotCommand> & control_targets)
  {
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    std::vector<crane_msgs::msg::RobotCommand> control_targets;
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

    if (required_wall_kick) {
      crane_msgs::msg::RobotCommand target;
      target.robot_id = robots.front().robot_id;
      target.chip_enable = false;
      target.kick_power = 0.5;
      target.dribble_power = 0.0;
      return control_targets;
    }

    for (auto r : robots | boost::adaptors::indexed()) {
      crane_msgs::msg::RobotCommand target;
      auto robot = world_model_->getRobot(r.value());
      // common param
      target.disable_placement_avoidance = true;
      target.robot_id = robot->id;
      target.chip_enable = false;
      target.dribble_power = 0.0;
      target.kick_power = 0.0;

      if (dribble_mode) {
        if (r.index() > 0) {
          target.disable_placement_avoidance = false;
        } else {
        }
      }

      // TODO: implement
      target.motion_mode_enable = false;
      target.target.x = 0.0;
      target.target.y = 0.0;
      target.target.theta = 0.0;  // omega

      control_targets.emplace_back(target);
    }
    return control_targets;
  }
  double getRoleScore(std::shared_ptr<RobotInfo> robot) override
  {
    // the nearest to the ball first
    return 100. / world_model_->getSquareDistanceFromRobotToBall({true, robot->id});
  }

private:
  BallPlacementState state_;
};

}  // namespace crane
#endif  // CRANE_BALL_PLACEMENT_PLANNER__BALL_PLACEMENT_PLANNER_HPP_
