// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SENDER_BASE_HPP_
#define CRANE_SENDER__SENDER_BASE_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/parameter_with_event.hpp>
#include <crane_basics/pid_controller.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
class SenderBase : public rclcpp::Node
{
public:
  explicit SenderBase(const std::string name, const rclcpp::NodeOptions & options)
  : Node(name, options),
    sub_commands(create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands", 10,
      [this](const crane_msgs::msg::RobotCommands & msg) { callback(msg); })),
    clock(RCL_ROS_TIME)
  {
    declare_parameter<bool>("no_movement", false);
    get_parameter("no_movement", no_movement);

    declare_parameter<double>("delay_s", 0.0);
    get_parameter("delay_s", delay_s);

    declare_parameter<double>("kick_power_limit_straight", 1.0);
    get_parameter("kick_power_limit_straight", kick_power_limit_straight);

    declare_parameter<double>("kick_power_limit_chip", 1.0);
    get_parameter("kick_power_limit_chip", kick_power_limit_chip);

    declare_parameter<double>("latency_ms", 0.0);
    get_parameter("latency_ms", current_latency_ms);

    world_model = std::make_shared<WorldModelWrapper>(*this);
  }

protected:
  const rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_commands;

  std::array<PIDController, 20> theta_controllers;

  virtual void sendCommands(const crane_msgs::msg::RobotCommands & msg) = 0;

  double delay_s;

  WorldModelWrapper::SharedPtr world_model;

  rclcpp::Clock clock;

  bool no_movement;

private:
  double current_latency_ms = 0.0;

  double kick_power_limit_straight;

  double kick_power_limit_chip;

  void callback(const crane_msgs::msg::RobotCommands & msg)
  {
    if (not world_model->hasUpdated()) {
      return;
    }

    auto now = clock.now();

    crane_msgs::msg::RobotCommands preprocessed_msg = msg;

    for (auto & command : preprocessed_msg.robot_commands) {
      command.latency_ms = current_latency_ms;
      command.kick_power = std::clamp(command.kick_power, 0.f, [this, command]() -> float {
        return command.chip_enable ? kick_power_limit_chip : kick_power_limit_straight;
      }());
      command.dribble_power = std::clamp(command.dribble_power, 0.f, 1.f);

      try {
        auto elapsed_time = now - world_model->getOurRobot(command.robot_id)->detection_stamp;
        command.elapsed_time_ms_since_last_vision = elapsed_time.nanoseconds() / 1e6;
      } catch (...) {
        std::cerr << "Error: Failed to get elapsed time of vision from world_model" << std::endl;
        command.elapsed_time_ms_since_last_vision = 0;
      }

      switch (command.control_mode) {
        case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE:
          // NOT SUPPORTED NOW
          break;
        case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
          break;
        case crane_msgs::msg::RobotCommand::VELOCITY_TARGET_MODE: {
        } break;
        case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE:
          break;
      }
    }

    if (no_movement) {
      for (auto & command : preprocessed_msg.robot_commands) {
        command.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
        command.omega_limit = 0.0f;
        command.simple_velocity_target_mode.clear();
        crane_msgs::msg::SimpleVelocityTargetMode target;
        target.target_vx = 0.0f;
        target.target_vy = 0.0f;
        target.speed_limit_at_target = 0.0f;
        command.simple_velocity_target_mode.push_back(target);
        command.chip_enable = false;
        command.dribble_power = 0.0;
        command.kick_power = 0.0;
      }
    }
    sendCommands(preprocessed_msg);
  }
};
}  // namespace crane

#endif  // CRANE_SENDER__SENDER_BASE_HPP_
