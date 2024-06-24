// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__SIMPLE_AVOID_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__SIMPLE_AVOID_PLANNER_HPP_

#include <crane_basics/pid_controller.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <memory>
#include <vector>

namespace crane
{
class SimpleAvoidPlanner
{
public:
  explicit SimpleAvoidPlanner(rclcpp::Node & node);

  std::vector<Point> getAvoidancePoints(
    const Point & point, const Circle & circle, const double offset);

  std::vector<Point> getAvoidancePoints(
    const Point & point, const Point & target, const double offset);

  std::vector<Point> getAvoidancePoints(const Point & point, const Box & box, const double offset);

  std::vector<Point> getAvoidancePoints(
    const Point & point, const Capsule & capsule, const double offset);

  std::optional<Point> getAvoidancePoint(
    const crane_msgs::msg::RobotCommand & command, WorldModelWrapper::SharedPtr world_model);

  std::optional<Point> getAvoidancePoint(
    std::shared_ptr<RobotInfo> from_robot, crane_msgs::msg::LocalPlannerConfig config, Point to,
    WorldModelWrapper::SharedPtr world_model, int depth = 0);

  void filterAvoidancePointsByPlace(
    std::vector<Point> & points, crane_msgs::msg::LocalPlannerConfig config,
    WorldModelWrapper::SharedPtr world_model);

  void filterAvoidancePointsByPath(
    std::vector<Point> & points, std::shared_ptr<RobotInfo> robot,
    crane_msgs::msg::LocalPlannerConfig config, Point from,
    WorldModelWrapper::SharedPtr world_model);

  crane_msgs::msg::RobotCommands calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg, WorldModelWrapper::SharedPtr world_model);

private:
  std::array<PIDController, 20> vx_controllers;
  std::array<PIDController, 20> vy_controllers;

  double NON_RVO_MAX_VEL = 4.0;
  double NON_RVO_P_GAIN = 4.0;
  double NON_RVO_I_GAIN = 0.0;
  double NON_RVO_D_GAIN = 0.0;

  rclcpp::Logger logger;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__SIMPLE_AVOID_PLANNER_HPP_
