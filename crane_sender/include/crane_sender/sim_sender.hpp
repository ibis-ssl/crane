// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SIM_SENDER_HPP_
#define CRANE_SENDER__SIM_SENDER_HPP_

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "crane_msgs/msg/robot_commands.hpp"
#include "crane_sender/sender_base.hpp"
#include "robocup_ssl_msgs/msg/commands.hpp"
#include "robocup_ssl_msgs/msg/replacement.hpp"
#include "robocup_ssl_msgs/msg/robot_command.hpp"

namespace crane
{

class SimSenderComponent : public SenderBase
{
public:
  SimSenderComponent(const rclcpp::NodeOptions & options)
  : SenderBase("sim_sender", options),
    pub_commands(create_publisher<robocup_ssl_msgs::msg::Commands>("/commands", 10))
  {
    //    sub_replacement_ = this->create_subscription<robocup_ssl_msgs::msg::Replacement>(
    //      "sim_sender/replacements", 10, std::bind(&SimSender::send_replacement, this, std::placeholders::_1));
  }

  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    const double MAX_KICK_SPEED = 8.0;  // m/s
    robocup_ssl_msgs::msg::Commands commands;
    commands.isteamyellow = msg.is_yellow;
    commands.timestamp = msg.header.stamp.sec;

    for (const auto command : msg.robot_commands) {
      robocup_ssl_msgs::msg::RobotCommand cmd;
      cmd.set__id(command.robot_id);

      // 走行速度
      cmd.set__veltangent(command.target_velocity.x);
      cmd.set__velnormal(command.target_velocity.y);
      cmd.set__velangular(command.target_velocity.theta);

      // キック速度
      double kick_speed = command.kick_power * MAX_KICK_SPEED;
      cmd.set__kickspeedx(kick_speed);

      // チップキック
      if (command.chip_enable) {
        cmd.set__kickspeedz(kick_speed);
      } else {
        cmd.set__kickspeedz(0);
      }

      // ドリブル
      cmd.set__spinner(command.dribble_power > 0);

      // タイヤ個別に速度設定しない
      cmd.set__wheelsspeed(false);

      if (no_movement) {
        cmd.set__spinner(false);
      }
      commands.robot_commands.emplace_back(cmd);
    }

    pub_commands->publish(commands);
  }

  //  void send_replacement(const consai2r2_msgs::msg::Replacements::SharedPtr msg) const
  //  {
  //
  //    auto replacement = new grSim_Replacement();
  //    if(msg->ball.is_enabled){
  //        auto replace_ball = new grSim_BallReplacement();
  //        replace_ball->set_x(msg->ball.x);
  //        replace_ball->set_y(msg->ball.y);
  //        replace_ball->set_vx(msg->ball.vx);
  //        replace_ball->set_vy(msg->ball.vy);
  //        replacement->set_allocated_ball(replace_ball);
  //    }
  //    for(auto robot : msg->robots){
  //        auto replace_robot = replacement->add_robots();
  //        replace_robot->set_x(robot.x);
  //        replace_robot->set_y(robot.y);
  //        replace_robot->set_dir(robot.dir);
  //        replace_robot->set_id(robot.id);
  //        replace_robot->set_yellowteam(robot.yellowteam);
  //        replace_robot->set_turnon(robot.turnon);
  //    }
  //    auto packet = new grSim_Packet();
  //    packet->set_allocated_replacement(replacement);
  //
  //    std::cout << "output" << std::endl;
  //    std::string output;
  //    packet->SerializeToString(&output);
  //    std::cout << output << std::endl;
  //    udp_sender_->send(output);
  //  }

  //  rclcpp::Subscription<consai2r2_msgs::msg::Replacements>::SharedPtr sub_replacement;

  const rclcpp::Publisher<robocup_ssl_msgs::msg::Commands>::SharedPtr pub_commands;

  std::array<float, 20> vel;
};
}  // namespace crane
#endif  // CRANE_SENDER__SIM_SENDER_HPP_
