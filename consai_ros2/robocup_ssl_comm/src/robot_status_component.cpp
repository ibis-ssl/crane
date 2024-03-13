// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "robocup_ssl_comm/robot_status_component.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{
GrSimRobotStatus::GrSimRobotStatus(const rclcpp::NodeOptions & options) : Node("vision", options)
{
  declare_parameter("multicast_address", "224.5.23.2");
  declare_parameter("blue_port", 10301);
  declare_parameter("yellow_port", 10302);
  yellow_receiver = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("multicast_address").get_value<std::string>(),
    get_parameter("yellow_port").get_value<int>());
  blue_receiver = std::make_unique<multicast::MulticastReceiver>(
    get_parameter("multicast_address").get_value<std::string>(),
    get_parameter("blue_port").get_value<int>());
  pub_robot_status_blue =
    create_publisher<robocup_ssl_msgs::msg::RobotsStatus>("/robots_status/blue", 10);
  pub_robot_status_yellow =
    create_publisher<robocup_ssl_msgs::msg::RobotsStatus>("/robots_status/yellow", 10);

  timer = create_wall_timer(10ms, std::bind(&Vision::on_timer, this));
}

void GrSimRobotStatus::on_timer()
{
  auto process = [](auto & receiver, auto & pub) {
    while (receiver->available()) {
      std::vector<char> buf(2048);
      const size_t size = receiver->receive(buf);

      if (size > 0) {
        RobotsStatus packet;
        packet.ParseFromString(std::string(buf.begin(), buf.end()));
        pub->publish(packet);
      }
    }
  };

  process(yellow_receiver, pub_robot_status_yellow);
  process(blue_receiver, pub_robot_status_blue);
}

void GrSimRobotStatus::publish_status(const Robots_Status & robots_status)
{
  auto statuses_msg = std::make_unique<robocup_ssl_msgs::msg::RobotsStatus>();

  for (const auto & status : robots_status.robots_status()) {
    robocup_ssl_msgs::msg::RobotStatus status_msg;
    status_msg.robot_id = status.robot_id();
    status_msg.infrared = status.infrared();
    status_msg.flat_kick = status.flat_kick();
    status_msg.chip_kick = status.chip_kick();

    statuses_msg->robots_status.push_back(std::move(status_msg));
  }
  pub_detection->publish(std::move(detection_msg));
}
}  // namespace robocup_ssl_comm

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::Vision)
