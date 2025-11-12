// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROBOCUP_SSL_COMM__GRSIM_COMPONENT_HPP_
#define ROBOCUP_SSL_COMM__GRSIM_COMPONENT_HPP_

#include <robocup_ssl_comm/visibility_control.h>
#include <robocup_ssl_msgs/grSim_Commands.pb.h>
#include <robocup_ssl_msgs/grSim_Replacement.pb.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_comm/udp_sender.hpp>
#include <robocup_ssl_msgs/msg/gr_sim_commands.hpp>
#include <robocup_ssl_msgs/msg/gr_sim_replacement.hpp>
#include <robocup_ssl_msgs/msg/gr_sim_robot_command.hpp>
#include <robocup_ssl_msgs/msg/gr_sim_robot_replacement.hpp>
#include <robocup_ssl_msgs/robocup_ssl_msgs/conversions.hpp>

namespace robocup_ssl_comm
{
using Commands = robocup_ssl_msgs::msg::GrSimCommands;
using RobotCommand = robocup_ssl_msgs::msg::GrSimRobotCommand;
using Replacement = robocup_ssl_msgs::msg::GrSimReplacement;
using RobotReplacement = robocup_ssl_msgs::msg::GrSimRobotReplacement;

class GrSim : public rclcpp::Node
{
public:
  ROBOCUP_SSL_COMM_PUBLIC
  explicit GrSim(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  void callback_commands(const Commands::SharedPtr msg);

  void callback_single_command(const RobotCommand::SharedPtr msg);

  void callback_replacement(const Replacement::SharedPtr msg);

  void set_command(
    robocup_ssl::grSim_Robot_Command * robot_command, const RobotCommand & msg_robot_command);

  void set_robot_replacement(
    robocup_ssl::grSim_RobotReplacement * robot_replacement,
    const RobotReplacement & msg_robot_replacement);

  std::unique_ptr<udp_sender::UDPSender> sender;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Subscription<Commands>::SharedPtr sub_commands;

  rclcpp::Subscription<Replacement>::SharedPtr sub_replacement;
};

}  // namespace robocup_ssl_comm

#endif  // ROBOCUP_SSL_COMM__GRSIM_COMPONENT_HPP_
