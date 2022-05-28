// Copyright (c) 2022 ibis-ssl
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

#include <rclcpp/rclcpp.hpp>

#include "robocup_ssl_msgs/msg/ball_replacement.hpp"
#include "robocup_ssl_msgs/msg/replacement.hpp"
#include "robocup_ssl_msgs/msg/robot_replacement.hpp"

#define LOAD_ROBOT_POSE(ROBOT_NAME, ROBOT_ARRAY)                   \
  {                                                                \
    declare_parameter(#ROBOT_NAME, std::vector<double>(3, -20.0)); \
    auto pose = get_parameter(#ROBOT_NAME).as_double_array();      \
    uint32_t id = std::stoi(std::string(#ROBOT_NAME).substr(5));   \
    robocup_ssl_msgs::msg::RobotReplacement robot;                 \
    robot.x = pose[0];                                             \
    robot.y = pose[1];                                             \
    robot.dir = pose[2];                                           \
    robot.id = id;                                                 \
    if (pose[0] != -20.0) ROBOT_ARRAY.push_back(robot);            \
  }

class GrsimOperator : public rclcpp::Node
{
public:
  GrsimOperator() : Node("crane_grsim_operator")
  {
    // sub_commands_ = this->create_subscription<crane_msgs::msg::RobotCommands>("crane_commands", 10, std::bind(&GrsimOperator::send_commands, this, std::placeholders::_1));
    // sub_replacement_ = this->create_subscription<robocup_ssl_msgs::msg::Replacement>("sim_sender/", 10, std::bind(&GrsimOperator::send_replacement, this, std::placeholders::_1));
    pub_replacement_ =
      this->create_publisher<robocup_ssl_msgs::msg::Replacement>("replacement", 10);

    LOAD_ROBOT_POSE("yellow.robot1", yellow_robots_);
    LOAD_ROBOT_POSE("yellow.robot2", yellow_robots_);
    LOAD_ROBOT_POSE("yellow.robot3", yellow_robots_);
    LOAD_ROBOT_POSE("yellow.robot4", yellow_robots_);
    LOAD_ROBOT_POSE("yellow.robot5", yellow_robots_);
    LOAD_ROBOT_POSE("yellow.robot6", yellow_robots_);
    LOAD_ROBOT_POSE("yellow.robot7", yellow_robots_);
    LOAD_ROBOT_POSE("yellow.robot8", yellow_robots_);

    LOAD_ROBOT_POSE("blue.robot1", blue_robots_);
    LOAD_ROBOT_POSE("blue.robot2", blue_robots_);
    LOAD_ROBOT_POSE("blue.robot3", blue_robots_);
    LOAD_ROBOT_POSE("blue.robot4", blue_robots_);
    LOAD_ROBOT_POSE("blue.robot5", blue_robots_);
    LOAD_ROBOT_POSE("blue.robot6", blue_robots_);
    LOAD_ROBOT_POSE("blue.robot7", blue_robots_);
    LOAD_ROBOT_POSE("blue.robot8", blue_robots_);

    declare_parameter("ball.pos", std::vector<double>(2, -20.0));
    auto ball_pos = get_parameter("ball.pos").as_double_array();

    declare_parameter("ball.vel", std::vector<double>(2, 0.0));
    auto ball_vel = get_parameter("ball.vel").as_double_array();

    if (ball_pos[0] != -20.0) {
      robocup_ssl_msgs::msg::BallReplacement ball_replacement;
      ball_replacement.x.push_back(ball_pos[0]);
      ball_replacement.y.push_back(ball_pos[1]);
      ball_replacement.vx.push_back(ball_vel[0]);
      ball_replacement.vy.push_back(ball_vel[1]);
      ball_.push_back(ball_replacement);
    }

    publishReplacement();
  }

private:
  void publishReplacement()
  {
    robocup_ssl_msgs::msg::Replacement msg;
    for (auto ball : ball_) {
      msg.ball.push_back(ball);
    }
    for (auto & robot : yellow_robots_) {
      robot.yellowteam = true;
      msg.robots.push_back(robot);
    }
    for (auto & robot : blue_robots_) {
      robot.yellowteam = false;
      msg.robots.push_back(robot);
    }
    pub_replacement_->publish(msg);
  }
  std::vector<robocup_ssl_msgs::msg::RobotReplacement> yellow_robots_;
  std::vector<robocup_ssl_msgs::msg::RobotReplacement> blue_robots_;
  std::vector<robocup_ssl_msgs::msg::BallReplacement> ball_;
  rclcpp::Publisher<robocup_ssl_msgs::msg::Replacement>::SharedPtr pub_replacement_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GrsimOperator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
