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

#include "robocup_ssl_comm/grsim_component.hpp"

#include <robocup_ssl_msgs/grSim_Packet.pb.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{
using std::placeholders::_1;

GrSim::GrSim(const rclcpp::NodeOptions & options) : Node("grsim", options)
{
  sender = std::make_unique<crane::UDPSender>("127.0.0.1", 20011);

  sub_commands =
    create_subscription<Commands>("commands", 10, std::bind(&GrSim::callback_commands, this, _1));
  sub_replacement = create_subscription<Replacement>(
    "replacement", 10, std::bind(&GrSim::callback_replacement, this, _1));
}

void GrSim::callback_commands(const Commands::SharedPtr msg)
{
  robocup_ssl::grSim_Commands * commands = new robocup_ssl::grSim_Commands();

  commands->set_timestamp(msg->timestamp);
  commands->set_isteamyellow(msg->isteamyellow);
  for (const auto & msg_robot_command : msg->robot_commands) {
    set_command(commands->add_robot_commands(), msg_robot_command);
  }

  robocup_ssl::grSim_Packet packet;
  packet.set_allocated_commands(commands);

  std::string output;
  packet.SerializeToString(&output);
  sender->send(output);
}

void GrSim::callback_replacement(const Replacement::SharedPtr msg)
{
  robocup_ssl::grSim_Replacement * replacement = new robocup_ssl::grSim_Replacement();

  if (msg->has_field & msg->BALL_FIELD_SET) {
    robocup_ssl::grSim_BallReplacement * ball = new robocup_ssl::grSim_BallReplacement();
    const auto & msg_ball = msg->ball;
    if (msg_ball.has_field & msg_ball.X_FIELD_SET) {
      ball->set_x(msg_ball.x);
    }
    if (msg_ball.has_field & msg_ball.Y_FIELD_SET) {
      ball->set_y(msg_ball.y);
    }
    if (msg_ball.has_field & msg_ball.VX_FIELD_SET) {
      ball->set_vx(msg_ball.vx);
    }
    if (msg_ball.has_field & msg_ball.VY_FIELD_SET) {
      ball->set_vy(msg_ball.vy);
    }
    replacement->set_allocated_ball(ball);
  }

  for (const auto & msg_robot : msg->robots) {
    set_robot_replacement(replacement->add_robots(), msg_robot);
  }

  robocup_ssl::grSim_Packet packet;
  packet.set_allocated_replacement(replacement);

  std::string output;
  packet.SerializeToString(&output);
  sender->send(output);
}

void GrSim::set_command(
  robocup_ssl::grSim_Robot_Command * robot_command, const RobotCommand & msg_robot_command)
{
  // Use proto2ros Convert API
  robocup_ssl_msgs::conversions::Convert(msg_robot_command, robot_command);
}

void GrSim::set_robot_replacement(
  robocup_ssl::grSim_RobotReplacement * robot_replacement,
  const RobotReplacement & msg_robot_replacement)
{
  // Use proto2ros Convert API
  robocup_ssl_msgs::conversions::Convert(msg_robot_replacement, robot_replacement);
}

}  // namespace robocup_ssl_comm

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::GrSim)
