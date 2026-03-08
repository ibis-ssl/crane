// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <rclcpp/rclcpp.hpp>

#include "robocup_ssl_msgs/msg/gr_sim_ball_replacement.hpp"
#include "robocup_ssl_msgs/msg/gr_sim_replacement.hpp"
#include "robocup_ssl_msgs/msg/gr_sim_robot_replacement.hpp"

#define LOAD_ROBOT_POSE(ROBOT_NAME, ROBOT_ARRAY)                   \
  {                                                                \
    declare_parameter(#ROBOT_NAME, std::vector<double>(3, -20.0)); \
    auto pose = get_parameter(#ROBOT_NAME).as_double_array();      \
    uint32_t id = std::stoi(std::string(#ROBOT_NAME).substr(5));   \
    robocup_ssl_msgs::msg::GrSimRobotReplacement robot;            \
    robot.x = pose[0];                                             \
    robot.y = pose[1];                                             \
    robot.dir = pose[2];                                           \
    robot.id = id;                                                 \
    robot.has_field = 0;                                           \
    if (pose[0] != -20.0) ROBOT_ARRAY.push_back(robot);            \
  }

class GrsimOperator : public rclcpp::Node
{
public:
  GrsimOperator() : Node("crane_grsim_operator")
  {
    //    sub_commands_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
    //      "crane_commands", 10,
    //      std::bind(&GrsimOperator::send_commands, this, std::placeholders::_1));
    //    sub_replacement_ =
    //      this->create_subscription<robocup_ssl_msgs::msg::GrSimReplacement>(
    //        "sim_sender/", 10,
    //        std::bind(
    //          &GrsimOperator::send_replacement, this, std::placeholders::_1));
    pub_replacement =
      this->create_publisher<robocup_ssl_msgs::msg::GrSimReplacement>("replacement", 10);

    LOAD_ROBOT_POSE("yellow.robot1", yellow_robots)
    LOAD_ROBOT_POSE("yellow.robot2", yellow_robots)
    LOAD_ROBOT_POSE("yellow.robot3", yellow_robots)
    LOAD_ROBOT_POSE("yellow.robot4", yellow_robots)
    LOAD_ROBOT_POSE("yellow.robot5", yellow_robots)
    LOAD_ROBOT_POSE("yellow.robot6", yellow_robots)
    LOAD_ROBOT_POSE("yellow.robot7", yellow_robots)
    LOAD_ROBOT_POSE("yellow.robot8", yellow_robots)

    LOAD_ROBOT_POSE("blue.robot1", blue_robots)
    LOAD_ROBOT_POSE("blue.robot2", blue_robots)
    LOAD_ROBOT_POSE("blue.robot3", blue_robots)
    LOAD_ROBOT_POSE("blue.robot4", blue_robots)
    LOAD_ROBOT_POSE("blue.robot5", blue_robots)
    LOAD_ROBOT_POSE("blue.robot6", blue_robots)
    LOAD_ROBOT_POSE("blue.robot7", blue_robots)
    LOAD_ROBOT_POSE("blue.robot8", blue_robots)

    declare_parameter("ball.pos", std::vector<double>(2, -20.0));
    auto ball_pos = get_parameter("ball.pos").as_double_array();

    declare_parameter("ball.vel", std::vector<double>(2, 0.0));
    auto ball_vel = get_parameter("ball.vel").as_double_array();

    if (ball_pos[0] != -20.0) {
      robocup_ssl_msgs::msg::GrSimBallReplacement ball_replacement;
      ball_replacement.x = ball_pos[0];
      ball_replacement.y = ball_pos[1];
      ball_replacement.vx = ball_vel[0];
      ball_replacement.vy = ball_vel[1];
      ball_replacement.has_field = ball_replacement.X_FIELD_SET | ball_replacement.Y_FIELD_SET |
                                   ball_replacement.VX_FIELD_SET | ball_replacement.VY_FIELD_SET;
      ball = ball_replacement;
      has_ball = true;
    }

    publishReplacement();
  }

private:
  void publishReplacement()
  {
    robocup_ssl_msgs::msg::GrSimReplacement msg;
    msg.has_field = 0;
    if (has_ball) {
      msg.ball = ball;
      msg.has_field |= msg.BALL_FIELD_SET;
    }
    for (auto & robot : yellow_robots) {
      robot.yellowteam = true;
      robot.has_field |= robot.X_FIELD_SET | robot.Y_FIELD_SET | robot.DIR_FIELD_SET |
                         robot.ID_FIELD_SET | robot.YELLOWTEAM_FIELD_SET;
      msg.robots.push_back(robot);
    }
    for (auto & robot : blue_robots) {
      robot.yellowteam = false;
      robot.has_field |= robot.X_FIELD_SET | robot.Y_FIELD_SET | robot.DIR_FIELD_SET |
                         robot.ID_FIELD_SET | robot.YELLOWTEAM_FIELD_SET;
      msg.robots.push_back(robot);
    }
    pub_replacement->publish(msg);
  }

  std::vector<robocup_ssl_msgs::msg::GrSimRobotReplacement> yellow_robots;

  std::vector<robocup_ssl_msgs::msg::GrSimRobotReplacement> blue_robots;

  robocup_ssl_msgs::msg::GrSimBallReplacement ball;
  bool has_ball = false;

  rclcpp::Publisher<robocup_ssl_msgs::msg::GrSimReplacement>::SharedPtr pub_replacement;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GrsimOperator>());
  rclcpp::shutdown();
  return 0;
}
