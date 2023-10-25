// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__BALL_PLACEMENT_WITH_SKILL_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__BALL_PLACEMENT_WITH_SKILL_PLANNER_HPP_

#include <crane_planner_plugins/visibility_control.h>

#include <boost/range/adaptor/indexed.hpp>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/control_target.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <crane_robot_skills/get_ball_contact.hpp>
#include <crane_robot_skills/move_with_ball.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace crane
{
enum class BallPlacementState {
  GET_BALL_CONTACT,
  MOVE_WITH_BALL,
  CLEAR_BALL,
};

class BallPlacementWithSkillPlanner : public rclcpp::Node, public PlannerBase
{
public:
  COMPOSITION_PUBLIC
  explicit BallPlacementWithSkillPlanner(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("ball_placement_with_skill_planner", options),
    PlannerBase("ball_placement_with_skill", *this),
    state(BallPlacementState::GET_BALL_CONTACT)
  {
    RCLCPP_INFO(get_logger(), "initializing");
    //    addRobotSelectCallback([&]() { state = BallPlacementState::START; });
  }

  std::vector<crane_msgs::msg::RobotCommand> calculateControlTarget(
    const std::vector<RobotIdentifier> & robots) override
  {
    if (robots.size() != 1) {
      return {};
    }
    auto robot = world_model->getRobot(robots.front());
    static GetBallContact get_ball_contact(robot->id, world_model);
    static MoveWithBall move_with_ball(
      [&]() {
        Pose2D pose;
        pose.pos = placement_target + (world_model->ball.pos - placement_target).normalized() *
                                        robot->center_to_kicker().norm();
        pose.theta = getAngle(placement_target - world_model->ball.pos);
        return pose;
      }(),
      robot->id, world_model);

    crane::RobotCommandWrapper command(robot->id, world_model);

    if (state == BallPlacementState::GET_BALL_CONTACT) {
      std::cout << "GET_BALL_CONTACT" << std::endl;
      if (get_ball_contact.run(command) == SkillBase<>::Status::SUCCESS) {
        state = BallPlacementState::MOVE_WITH_BALL;
      }
    } else if (state == BallPlacementState::MOVE_WITH_BALL) {
      std::cout << "MOVE_WITH_BALL" << std::endl;
      auto status = move_with_ball.run(command);

      static int success_count = 0;
      if (status == SkillBase<>::Status::FAILURE) {
        state = BallPlacementState::GET_BALL_CONTACT;
        success_count = 0;
      } else if (status == SkillBase<>::Status::SUCCESS) {
        success_count++;
        if (success_count >= 20) {
          state = BallPlacementState::CLEAR_BALL;
        }
      } else {
        success_count = 0;
      }
    } else if (state == BallPlacementState::CLEAR_BALL) {
      std::cout << "CLEAR_BALL" << std::endl;
      command.setTargetPosition(
        placement_target - Vector2(cos(robot->pose.theta), sin(robot->pose.theta)) * 0.5);
      //      state = BallPlacementState::GET_BALL_CONTACT;
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

  Point placement_target = Point(0.0, 0.0);
};

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__BALL_PLACEMENT_WITH_SKILL_PLANNER_HPP_
