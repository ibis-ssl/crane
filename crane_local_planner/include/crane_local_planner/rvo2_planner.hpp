// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__RVO2_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__RVO2_PLANNER_HPP_

#include <rvo2_vendor/RVO/RVO.h>

#include <crane_basics/parameter_with_event.hpp>
#include <crane_basics/pid_controller.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <memory>

#include "planner_base.hpp"

// cspell: ignore OBST
namespace crane
{
class RVO2Planner : public LocalPlannerBase
{
public:
  explicit RVO2Planner(rclcpp::Node & node);

  void reflectWorldToRVOSim(const crane_msgs::msg::RobotCommands & msg);

  crane_msgs::msg::RobotCommands extractRobotCommandsFromRVOSim(
    const crane_msgs::msg::RobotCommands & msg);

  crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg) override;

  void overrideTargetPosition(crane_msgs::msg::RobotCommands & msg);

private:
  std::unique_ptr<RVO::RVOSimulator> rvo_sim;

  crane_msgs::msg::RobotCommands pre_commands;

  RVO::Vector2 toRVO(const Point & point) { return RVO::Vector2(point.x(), point.y()); }

  Point toPoint(const RVO::Vector2 & vector) { return Point(vector.x(), vector.y()); }

  float RVO_TIME_STEP = 1.0 / 60.0f;
  float RVO_NEIGHBOR_DIST = 2.0f;
  int RVO_MAX_NEIGHBORS = 5;
  float RVO_TIME_HORIZON = 1.f;
  float RVO_TIME_HORIZON_OBST = 1.f;
  float RVO_RADIUS = 0.09f;
  float RVO_MAX_SPEED = 10.0f;

  float RVO_TRAPEZOIDAL_MAX_ACC = 8.0;
  float RVO_TRAPEZOIDAL_FRAME_RATE = 60;
  float RVO_TRAPEZOIDAL_MAX_SPEED = 4.0;

  double MAX_VEL = 4.0;
  double ACCELERATION = 4.0;
  // 減速度は加速度の何倍にするかという係数
  ParameterWithEvent<double> deceleration_factor;

  std::array<PIDController, 20> vx_controllers;
  std::array<PIDController, 20> vy_controllers;

  ParameterWithEvent<double> p_gain;
  ParameterWithEvent<double> i_gain;
  ParameterWithEvent<double> d_gain;

  double I_SATURATION = 0.0;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__RVO2_PLANNER_HPP_
