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

#include <boost/asio.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "crane_msgs/msg/robot_commands.hpp"
#include "consai2r2_msgs/msg/replacements.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "consai2r2_protobuf/grSim_Commands.pb.h"
#include "consai2r2_protobuf/grSim_Packet.pb.h"
#include "consai2r2_protobuf/grSim_Replacement.pb.h"


using std::placeholders::_1;

namespace asio = boost::asio;

class UDPSender
{
public:
  UDPSender(const std::string & ip, const int port)
  : socket_(io_service_, asio::ip::udp::endpoint(asio::ip::udp::v4(), 0))
  {
    asio::ip::udp::resolver resolver(io_service_);
    asio::ip::udp::resolver::query query(ip, std::to_string(port));
    endpoint_ = *resolver.resolve(query);
  }

  void send(const std::string & str)
  {
    socket_.send_to(asio::buffer(str), endpoint_);
  }

private:
  asio::io_service io_service_;
  asio::ip::udp::endpoint endpoint_;
  asio::ip::udp::socket socket_;
};

class SimSender : public rclcpp::Node
{
public:
  SimSender()
  : Node("consai2r2_sim_sender")
  {
    this->declare_parameter("grsim_addr", "127.0.0.1");
    this->declare_parameter("grsim_port", 20011);
    auto host = this->get_parameter("grsim_addr").as_string();
    auto port = this->get_parameter("grsim_port").as_int();

    sub_commands_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "robot_commands", 10, std::bind(&SimSender::send_commands, this, std::placeholders::_1));
    sub_replacement_ = this->create_subscription<consai2r2_msgs::msg::Replacements>(
      "sim_sender/replacements", 10, std::bind(&SimSender::send_replacement, this, std::placeholders::_1));
    udp_sender_ = std::make_shared<UDPSender>(host, port);
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
    grSim_Commands * packet_commands = new grSim_Commands();

    packet_commands->set_timestamp(msg->header.stamp.sec);
    packet_commands->set_isteamyellow(msg->is_yellow);

    for (auto command : msg->robot_commands) {
      grSim_Robot_Command * robot_command = packet_commands->add_robot_commands();
      robot_command->set_id(command.robot_id);

      // 走行速度
      robot_command->set_veltangent(command.target.x);
      robot_command->set_velnormal(command.target.y);

      float diff = getAngleDiff(command.current_theta, command.target.theta);
      robot_command->set_velangular(-10.0*diff);

      // キック速度
      double kick_speed = command.kick_power * MAX_KICK_SPEED;
      robot_command->set_kickspeedx(kick_speed);

      // チップキック
      if (command.chip_enable) {
        robot_command->set_kickspeedz(kick_speed);
      } else {
        robot_command->set_kickspeedz(0);
      }

      // ドリブル
      robot_command->set_spinner(command.dribble_power > 0);

      // タイヤ個別に速度設定しない
      robot_command->set_wheelsspeed(false);
    }

    grSim_Packet packet;
    packet.set_allocated_commands(packet_commands);

    std::string output;
    packet.SerializeToString(&output);
    std::cout << output << std::endl;
    udp_sender_->send(output);
  }

  void send_replacement(const consai2r2_msgs::msg::Replacements::SharedPtr msg) const
  {

    auto replacement = new grSim_Replacement();
    if(msg->ball.is_enabled){
        auto replace_ball = new grSim_BallReplacement();
        replace_ball->set_x(msg->ball.x);
        replace_ball->set_y(msg->ball.y);
        replace_ball->set_vx(msg->ball.vx);
        replace_ball->set_vy(msg->ball.vy);
        replacement->set_allocated_ball(replace_ball);
    }
    for(auto robot : msg->robots){
        auto replace_robot = replacement->add_robots();
        replace_robot->set_x(robot.x);
        replace_robot->set_y(robot.y);
        replace_robot->set_dir(robot.dir);
        replace_robot->set_id(robot.id);
        replace_robot->set_yellowteam(robot.yellowteam);
        replace_robot->set_turnon(robot.turnon);
    }
    auto packet = new grSim_Packet();
    packet->set_allocated_replacement(replacement);

    std::cout << "output" << std::endl;
    std::string output;
    packet->SerializeToString(&output);
    std::cout << output << std::endl;
    udp_sender_->send(output);
  }

  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_commands_;
  rclcpp::Subscription<consai2r2_msgs::msg::Replacements>::SharedPtr sub_replacement_;
  std::shared_ptr<UDPSender> udp_sender_;
  std::array<float, 11> vel;
};

void test(std::shared_ptr<SimSender> node){
    auto replacement = std::make_shared<consai2r2_msgs::msg::Replacements>();
    replacement->ball.is_enabled = true;
    replacement->ball.x = 0;
    replacement->ball.y = 0;
    replacement->ball.vx = 1.0;
    replacement->ball.vy = 1.0;

    consai2r2_msgs::msg::ReplaceRobot robot_msg;
    robot_msg.x = 0.0;
    robot_msg.y = 0.0;
    robot_msg.id = 1;
    robot_msg.dir = 1.57;
    robot_msg.turnon = true;
    replacement->robots.push_back(robot_msg);
    node->send_replacement(replacement);
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
