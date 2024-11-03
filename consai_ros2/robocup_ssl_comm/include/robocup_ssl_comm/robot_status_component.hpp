// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ROBOCUP_SSL_COMM__ROBOT_STATUS_COMPONENT_HPP_
#define ROBOCUP_SSL_COMM__ROBOT_STATUS_COMPONENT_HPP_

#include <robocup_ssl_msgs/grSim_Robotstatus.pb.h>
#include <robocup_ssl_msgs/ssl_vision_detection.pb.h>
#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/robots_status.hpp>

#include "multicast.hpp"
#include "visibility_control.h"

namespace robocup_ssl_comm
{
class GrSimRobotStatus : public rclcpp::Node
{
public:
  ROBOCUP_SSL_COMM_PUBLIC
  explicit GrSimRobotStatus(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  robocup_ssl_msgs::msg::RobotsStatus get_status_msg(const Robots_Status & robots_status);

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<multicast::MulticastReceiver> yellow_receiver;

  std::unique_ptr<multicast::MulticastReceiver> blue_receiver;

  rclcpp::Publisher<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr pub_robots_status_blue;

  rclcpp::Publisher<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr pub_robots_status_yellow;
};

}  // namespace robocup_ssl_comm

#endif  // ROBOCUP_SSL_COMM__ROBOT_STATUS_COMPONENT_HPP_
