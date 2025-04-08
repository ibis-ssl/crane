// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <robocup_ssl_msgs/grSim_Robotstatus.pb.h>
#include <robocup_ssl_msgs/ssl_vision_detection.pb.h>
#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <chrono>
#include <crane_basics/multicast.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/robots_status.hpp>
#include <string>
#include <utility>
#include <vector>

namespace crane
{
class GrSimRobotStatusNode : public rclcpp::Node
{
public:
  explicit GrSimRobotStatusNode(const rclcpp::NodeOptions & options) : Node("vision", options)
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
    pub_robots_status_blue =
      create_publisher<robocup_ssl_msgs::msg::RobotsStatus>("/robots_status/blue", 10);
    pub_robots_status_yellow =
      create_publisher<robocup_ssl_msgs::msg::RobotsStatus>("/robots_status/yellow", 10);

    using std::chrono_literals::operator""ms;
    timer = rclcpp::create_timer(
      this, get_clock(), 10ms, std::bind(&GrSimRobotStatusNode::on_timer, this));
  }

protected:
  void on_timer()
  {
    auto process = [this](auto & receiver, auto & pub) {
      while (receiver->available()) {
        std::vector<char> buf(2048);
        const size_t size = receiver->receive(buf);

        if (size > 0) {
          Robots_Status packet;
          packet.ParseFromString(std::string(buf.begin(), buf.end()));
          pub->publish(get_status_msg(packet));
        }
      }
    };

    process(yellow_receiver, pub_robots_status_yellow);
    process(blue_receiver, pub_robots_status_blue);
  }

private:
  robocup_ssl_msgs::msg::RobotsStatus get_status_msg(const Robots_Status & robots_status)
  {
    auto statuses_msg = robocup_ssl_msgs::msg::RobotsStatus();

    for (const auto & status : robots_status.robots_status()) {
      robocup_ssl_msgs::msg::RobotStatus status_msg;
      status_msg.robot_id = status.robot_id();
      status_msg.infrared = status.infrared();
      status_msg.flat_kick = status.flat_kick();
      status_msg.chip_kick = status.chip_kick();

      statuses_msg.robots_status.push_back(std::move(status_msg));
    }

    return statuses_msg;
  }

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<multicast::MulticastReceiver> yellow_receiver;

  std::unique_ptr<multicast::MulticastReceiver> blue_receiver;

  rclcpp::Publisher<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr pub_robots_status_blue;

  rclcpp::Publisher<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr pub_robots_status_yellow;
};
}  // namespace crane

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::GrSimRobotStatusNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
