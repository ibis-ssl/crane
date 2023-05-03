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
#include <string>

#include "crane_msgs/msg/robot_commands.hpp"
#include "robocup_ssl_msgs/msg/commands.hpp"
#include "robocup_ssl_msgs/msg/replacement.hpp"
#include "robocup_ssl_msgs/msg/robot_command.hpp"
#include "std_msgs/msg/string.hpp"

namespace crane
{

class PIDController
{
public:
  PIDController() = default;
  void setGain(float kp, float ki, float kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    error_prev_ = 0.0f;
  }
  float update(float error, float dt)
  {
    float p = kp_ * error;
    float i = ki_ * (error + error_prev_) * dt / 2.0f;
    float d = kd_ * (error - error_prev_) / dt;
    error_prev_ = error;
    return p + i + d;
  }

private:
  float kp_;
  float ki_;
  float kd_;
  float error_prev_;
};

class SimSenderComponent : public rclcpp::Node
{
public:
  SimSenderComponent(const rclcpp::NodeOptions & options) : Node("sim_sender", options)
  {
    declare_parameter<bool>("no_movement", false);
    get_parameter("no_movement", no_movement_);

    using std::placeholders::_1;
    sub_commands_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands", 10,
      std::bind(&SimSenderComponent::send_commands, this, std::placeholders::_1));
    //    sub_replacement_ = this->create_subscription<robocup_ssl_msgs::msg::Replacement>(
    //      "sim_sender/replacements", 10, std::bind(&SimSender::send_replacement, this, std::placeholders::_1));
    pub_commands_ = this->create_publisher<robocup_ssl_msgs::msg::Commands>("/commands", 10);

    for (auto & controller : theta_controllers) {
      controller.setGain(4, 0.00, 0.1);
    }
  }

  //private:
  float normalizeAngle(float angle_rad) const
  {
    while (angle_rad > M_PI) {
      angle_rad -= 2.0f * M_PI;
    }
    while (angle_rad < -M_PI) {
      angle_rad += 2.0f * M_PI;
    }
    return angle_rad;
  }

  float getAngleDiff(float angle_rad1, float angle_rad2) const
  {
    angle_rad1 = normalizeAngle(angle_rad1);
    angle_rad2 = normalizeAngle(angle_rad2);
    if (abs(angle_rad1 - angle_rad2) > M_PI) {
      if (angle_rad1 - angle_rad2 > 0) {
        return angle_rad1 - angle_rad2 - 2.0f * M_PI;
      } else {
        return angle_rad1 - angle_rad2 + 2.0f * M_PI;
      }
    } else {
      return angle_rad1 - angle_rad2;
    }
  }

  void send_commands(const crane_msgs::msg::RobotCommands::SharedPtr msg)
  {
    const double MAX_KICK_SPEED = 8.0;  // m/s
    robocup_ssl_msgs::msg::Commands commands;
    commands.isteamyellow = msg->is_yellow;
    commands.timestamp = msg->header.stamp.sec;

    for (auto command : msg->robot_commands) {
      robocup_ssl_msgs::msg::RobotCommand cmd;
      cmd.set__id(command.robot_id);

      // 走行速度
      // フィールド座標系からロボット座標系に変換
      cmd.set__veltangent(
        command.target.x * cos(-command.current_theta) -
        command.target.y * sin(-command.current_theta));
      cmd.set__velnormal(
        command.target.x * sin(-command.current_theta) +
        command.target.y * cos(-command.current_theta));

      float omega = theta_controllers.at(command.robot_id)
                      .update(getAngleDiff(command.current_theta, command.target.theta), 0.033);
      cmd.set__velangular(omega);

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

      if (no_movement_) {
        cmd.set__velangular(0);
        cmd.set__velnormal(0);
        cmd.set__veltangent(0);
        cmd.set__kickspeedx(0);
        cmd.set__kickspeedz(0);
        cmd.set__spinner(false);
      }
      commands.robot_commands.emplace_back(cmd);
    }

    pub_commands_->publish(commands);
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

  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_commands_;
  //  rclcpp::Subscription<consai2r2_msgs::msg::Replacements>::SharedPtr sub_replacement_;
  rclcpp::Publisher<robocup_ssl_msgs::msg::Commands>::SharedPtr pub_commands_;
  std::array<float, 11> vel;
  std::array<PIDController, 11> theta_controllers;
  bool no_movement_;
};

}  // namespace crane
#endif  // CRANE_SENDER__SIM_SENDER_HPP_
