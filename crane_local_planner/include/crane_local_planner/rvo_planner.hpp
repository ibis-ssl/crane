// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__RVO_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__RVO_PLANNER_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <memory>

#include "RVO.h"

// cspell: ignore OBST
namespace crane
{
class RVOPlanner
{
public:
  explicit RVOPlanner(rclcpp::Node & node);

  void reflectWorldToRVOSim(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model);

  crane_msgs::msg::RobotCommands extractRobotCommandsFromRVOSim(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model);

  crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model);

private:
  std::unique_ptr<RVO::RVOSimulator> rvo_sim;

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
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__RVO_PLANNER_HPP_
