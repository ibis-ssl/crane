// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__RVO2_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__RVO2_PLANNER_HPP_

#include <rvo2_vendor/RVO/RVO.h>

#include <crane_comm/parameter_with_event.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/position_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/velocity_commands.hpp>
#include <crane_physics/pid_controller.hpp>
#include <memory>

#include "planner_base.hpp"

// cspell: ignore OBST
namespace crane
{
class RVO2Planner : public LocalPlannerBase
{
public:
  explicit RVO2Planner(rclcpp::Node & node);

  auto reflectWorldToRVOSim(crane_msgs::msg::PositionCommands & msg) -> void;

  auto extractVelocityCommandsFromRVOSim(
    const crane_msgs::msg::PositionCommands & msg, double theta_offset)
    -> crane_msgs::msg::VelocityCommands;

  auto calculateRobotCommand(const crane_msgs::msg::PositionCommands & msg, double theta_offset)
    -> crane_msgs::msg::VelocityCommands override;

  auto overrideTargetPosition(crane_msgs::msg::PositionCommands & msg) -> void;

private:
  auto adjustForPenaltyAreaAvoidance(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::PositionCommand & command) const -> void;

  auto adjustForBallAvoidance(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::PositionCommand & command) const -> void;

  auto adjustForPlacementAvoidance(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::PositionCommand & command) const -> void;

  std::unique_ptr<RVO::RVOSimulator> rvo_sim;

  crane_msgs::msg::PositionCommands pre_commands;

  auto toRVO(const Point & point) -> RVO::Vector2 { return RVO::Vector2(point.x(), point.y()); }

  auto toPoint(const RVO::Vector2 & vector) -> Point { return Point(vector.x(), vector.y()); }

  float RVO_TIME_STEP = 1.0 / 60.0f;
  float RVO_NEIGHBOR_DIST = 2.0f;
  int RVO_MAX_NEIGHBORS = 5;
  float RVO_TIME_HORIZON = 1.f;
  float RVO_TIME_HORIZON_OBST = 1.f;
  float RVO_RADIUS = 0.09f;
  float RVO_MAX_SPEED = 10.0f;

  double MAX_VEL = 4.0;
  double ACCELERATION = 4.0;
  double STOP_STATE_MAX_VELOCITY = 1.0;
  // 加速度は減速度の何倍にするかという係数
  ParameterWithEvent<double> acceleration_factor;

  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr sub_feedback_array;

  crane_msgs::msg::RobotFeedbackArray latest_feedback;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__RVO2_PLANNER_HPP_
