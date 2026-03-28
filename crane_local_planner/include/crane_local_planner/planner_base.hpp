// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_
#define CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_command.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <memory>

namespace crane
{
class LocalPlannerBase
{
public:
  LocalPlannerBase(const std::string & name, rclcpp::Node & node)
  : visualizer(std::make_shared<VisualizerMessageBuilder>("planner/" + name))
  {
    world_model = std::make_shared<WorldModelWrapper>(node);

    // 経路計画用の減速度パラメータを読み込み
    node.declare_parameter("planning_deceleration", 2.5);
    planning_deceleration = node.get_parameter("planning_deceleration").as_double();

    // 経路計画用の加速度パラメータを読み込み（減速度とは別に設定）
    node.declare_parameter("planning_acceleration", 5.0);
    planning_acceleration = node.get_parameter("planning_acceleration").as_double();
  }
  virtual auto calculateRobotCommand(
    const crane_msgs::msg::RobotCommands & msg, double theta_offset)
    -> crane_msgs::msg::RobotCommands = 0;

  auto getWorldModel() const -> WorldModelWrapper::SharedPtr { return world_model; }

  auto getVisualizer() const -> VisualizerMessageBuilder::SharedPtr { return visualizer; }

  static auto resolveMaxAccelerationFactors(
    crane_msgs::msg::RobotCommand & command, const float default_max_acceleration) -> double
  {
    crane_msgs::msg::NamedFloat minimum_max_acceleration_factor;
    minimum_max_acceleration_factor.set__name("default").set__value(default_max_acceleration);
    for (const auto & factor : command.local_planner_config.max_acceleration_factors) {
      if (minimum_max_acceleration_factor.value > factor.value) {
        minimum_max_acceleration_factor = factor;
      }
    }
    command.local_planner_config.final_planned_max_acceleration = minimum_max_acceleration_factor;
    return command.local_planner_config.final_planned_max_acceleration.value;
  }

  static auto resolveMaxVelocityFactors(
    crane_msgs::msg::RobotCommand & command, const float default_max_velocity) -> double
  {
    crane_msgs::msg::NamedFloat minimum_max_velocity_factor;
    minimum_max_velocity_factor.set__name("default").set__value(default_max_velocity);
    for (const auto & factor : command.local_planner_config.max_velocity_factors) {
      if (minimum_max_velocity_factor.value > factor.value) {
        minimum_max_velocity_factor = factor;
      }
    }
    command.local_planner_config.final_planned_max_velocity = minimum_max_velocity_factor;
    return command.local_planner_config.final_planned_max_velocity.value;
  }

protected:
  VisualizerMessageBuilder::SharedPtr visualizer;

  WorldModelWrapper::SharedPtr world_model;

  // 経路計画用の減速度パラメータ
  double planning_deceleration;
  // 経路計画用の加速度パラメータ（加速フェーズに使用）
  double planning_acceleration;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__PLANNER_BASE_HPP_
