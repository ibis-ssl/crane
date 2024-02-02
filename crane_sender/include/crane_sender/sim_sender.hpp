// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__SIM_SENDER_HPP_
#define CRANE_SENDER__SIM_SENDER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/commands.hpp>
#include <robocup_ssl_msgs/msg/replacement.hpp>
#include <robocup_ssl_msgs/msg/robot_command.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "sender_base.hpp"

namespace crane
{

class SimSenderComponent : public SenderBase
{
public:
  SimSenderComponent(const rclcpp::NodeOptions & options)
  : SenderBase("sim_sender", options),
    pub_commands(create_publisher<robocup_ssl_msgs::msg::Commands>("/commands", 10))
  {
    declare_parameter<bool>("no_movement", false);
    get_parameter("no_movement", no_movement);

    // the parameters of the PID controller
    declare_parameter<float>("theta_kp", 4.0);
    declare_parameter<float>("theta_ki", 0.0);
    declare_parameter<float>("theta_kd", 0.1);
    float kp, ki, kd;
    get_parameter("theta_kp", kp);
    get_parameter("theta_ki", ki);
    get_parameter("theta_kd", kd);

    for (auto & controller : theta_controllers) {
      controller.setGain(kp, ki, kd);
    }

    declare_parameter("max_vel", MAX_VEL);
    MAX_VEL = get_parameter("max_vel").as_double();

    declare_parameter("p_gain", P_GAIN);
    P_GAIN = get_parameter("p_gain").as_double();
    declare_parameter("i_gain", I_GAIN);
    I_GAIN = get_parameter("i_gain").as_double();
    declare_parameter("d_gain", D_GAIN);
    D_GAIN = get_parameter("d_gain").as_double();

    for (auto & controller : vx_controllers) {
      controller.setGain(P_GAIN, I_GAIN, D_GAIN);
    }

    for (auto & controller : vy_controllers) {
      controller.setGain(P_GAIN, I_GAIN, D_GAIN);
    }
    //    sub_replacement_ = this->create_subscription<robocup_ssl_msgs::msg::Replacement>(
    //      "sim_sender/replacements", 10, std::bind(&SimSender::send_replacement, this, std::placeholders::_1));
  }

  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    if (checkNan(msg)) {
      return;
    }

    const double MAX_KICK_SPEED = 8.0;  // m/s
    robocup_ssl_msgs::msg::Commands commands;
    commands.isteamyellow = msg.is_yellow;
    commands.timestamp = msg.header.stamp.sec;

    for (const auto & command : msg.robot_commands) {
      robocup_ssl_msgs::msg::RobotCommand cmd;
      cmd.set__id(command.robot_id);

      Point robot_to_target;
      robot_to_target << command.target_pose.x - command.current_pose.x,
        command.target_pose.y - command.current_pose.y;

      double max_vel = command.local_planner_config.max_velocity > 0
                         ? command.local_planner_config.max_velocity
                         : MAX_VEL;
      //        double max_acc = command.local_planner_config.max_acceleration > 0? command.local_planner_config.max_acceleration : NON_RVO_GAIN;
      double max_omega = command.local_planner_config.max_omega > 0
                           ? command.local_planner_config.max_omega
                           : 600.0 * M_PI / 180;

      // 速度に変換する
      Velocity world_vel;
      world_vel << vx_controllers[command.robot_id].update(
        command.target_pose.x - command.current_pose.x, 1.f / 30.f),
        vy_controllers[command.robot_id].update(
          command.target_pose.y - command.current_pose.y, 1.f / 30.f);
      world_vel += world_vel.normalized() * command.local_planner_config.terminal_velocity;
      if (world_vel.norm() > max_vel) {
        world_vel = world_vel.normalized() * max_vel;
      }
      // 座標変換（ワールド->各ロボット）

      // 走行速度
      cmd.set__veltangent(
        world_vel.x() * cos(-command.current_pose.theta) -
        world_vel.y() * sin(-command.current_pose.theta));
      cmd.set__velnormal(
        world_vel.x() * sin(-command.current_pose.theta) +
        world_vel.y() * cos(-command.current_pose.theta));
      cmd.set__velangular(
        -theta_controllers.at(command.robot_id)
           .update(getAngleDiff(command.current_pose.theta, command.target_pose.theta), 0.033));

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

  bool checkNan(const crane_msgs::msg::RobotCommands & msg)
  {
    bool is_nan = false;
    for (const auto & command : msg.robot_commands) {
      if (std::isnan(command.terminal_target_velocity.x)) {
        std::cout << "id: " << command.robot_id << " terminal_target_velocity.x is nan"
                  << std::endl;
        is_nan = true;
      }
      if (std::isnan(command.terminal_target_velocity.y)) {
        std::cout << "id: " << command.robot_id << "terminal_target_velocity.y is nan" << std::endl;
        is_nan = true;
      }
      if (std::isnan(command.terminal_target_velocity.theta)) {
        std::cout << "id: " << command.robot_id << "terminal_target_velocity.theta is nan"
                  << std::endl;
        is_nan = true;
      }
      if (std::isnan(command.target_pose.x)) {
        std::cout << "id: " << command.robot_id << "target_pose.x is nan" << std::endl;
        is_nan = true;
      }
      if (std::isnan(command.target_pose.y)) {
        std::cout << "id: " << command.robot_id << "target_pose.y is nan" << std::endl;
        is_nan = true;
      }
      if (std::isnan(command.target_pose.theta)) {
        std::cout << "id: " << command.robot_id << "target_pose.theta is nan" << std::endl;
        is_nan = true;
      }
      if (std::isnan(command.kick_power)) {
        std::cout << "id: " << command.robot_id << "kick_power is nan" << std::endl;
        is_nan = true;
      }
      if (std::isnan(command.dribble_power)) {
        std::cout << "id: " << command.robot_id << "dribble_power is nan" << std::endl;
        is_nan = true;
      }
    }
    return is_nan;
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

  std::array<PIDController, 20> vx_controllers;

  std::array<PIDController, 20> vy_controllers;

  std::array<PIDController, 20> theta_controllers;

  bool no_movement;

  double MAX_VEL = 4.0;
  double P_GAIN = 4.0;
  double I_GAIN = 0.0;
  double D_GAIN = 0.0;
};
}  // namespace crane
#endif  // CRANE_SENDER__SIM_SENDER_HPP_
