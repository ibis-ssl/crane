// Copyright (c) 2019 SSL-Roots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <iostream>
#include <memory>
#include <string>

#include "crane_msgs/msg/robot_commands.hpp"
#include "robocup_ssl_msgs/msg/replacement.hpp"
#include "robocup_ssl_msgs/msg/commands.hpp"
#include "robocup_ssl_msgs/msg/robot_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SimSender : public rclcpp::Node
{
public:
  SimSender()
  : Node("crane_sim_sender")
  {
    sub_commands_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "crane_commands", 10, std::bind(&SimSender::send_commands, this, std::placeholders::_1));
//    sub_replacement_ = this->create_subscription<robocup_ssl_msgs::msg::Replacement>(
//      "sim_sender/replacements", 10, std::bind(&SimSender::send_replacement, this, std::placeholders::_1));
    pub_commands_ = this->create_publisher<robocup_ssl_msgs::msg::Commands>("commands", 10);
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

  void send_commands(const crane_msgs::msg::RobotCommands::SharedPtr msg) const
  {
    const double MAX_KICK_SPEED = 8.0;  // m/s
    robocup_ssl_msgs::msg::Commands commands;
    commands.isteamyellow = msg->is_yellow;
    commands.timestamp = msg->header.stamp.sec;

    for (auto command : msg->robot_commands) {
      robocup_ssl_msgs::msg::RobotCommand cmd;
      cmd.set__id(command.robot_id);

      // 走行速度
      cmd.set__veltangent(command.target.x);
      cmd.set__velnormal(command.target.y);

      float diff = getAngleDiff(command.current_theta, command.target.theta);
      cmd.set__velangular(-10.0*diff);

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
};

void test(std::shared_ptr<SimSender> node){
//    auto replacement = std::make_shared<robocup_ssl_msgs::msg::Replacement>();
//    replacement->ball.is_enabled = true;
//    replacement->ball.x = 0;
//    replacement->ball.y = 0;
//    replacement->ball.vx = 1.0;
//    replacement->ball.vy = 1.0;

//    consai2r2_msgs::msg::ReplaceRobot robot_msg;
//    robot_msg.x = 0.0;
//    robot_msg.y = 0.0;
//    robot_msg.id = 1;
//    robot_msg.dir = 1.57;
//    robot_msg.turnon = true;
//    replacement->robots.push_back(robot_msg);
//    node->send_replacement(replacement);
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimSender>();
//  test(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
