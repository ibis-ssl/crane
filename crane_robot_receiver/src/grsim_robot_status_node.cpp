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
#include <crane_comm/unicast.hpp>
#include <memory>
#include <mutex>
#include <range/v3/algorithm/find_if.hpp>
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

    const std::string address = get_parameter("multicast_address").get_value<std::string>();
    const int yellow_port = get_parameter("yellow_port").get_value<int>();
    const int blue_port = get_parameter("blue_port").get_value<int>();

    yellow_receiver =
      std::make_unique<crane::AsyncUdpReceiver>(asio_ctx_.io_context, address, yellow_port);
    blue_receiver =
      std::make_unique<crane::AsyncUdpReceiver>(asio_ctx_.io_context, address, blue_port);

    for (int i = 0; i < 20; ++i) {
      robocup_ssl_msgs::msg::RobotStatus robot_status;
      robot_status.robot_id = i;
      robot_status.flat_kick = true;
      yellow_status_msg.robots_status.emplace_back(robot_status);
      blue_status_msg.robots_status.emplace_back(robot_status);
    }

    auto make_recv_callback = [this](robocup_ssl_msgs::msg::RobotsStatus & target) {
      return [this, &target](const std::vector<char> & buf, size_t size) {
        if (size > 0) {
          robocup_ssl::Robots_Status packet;
          if (packet.ParseFromArray(buf.data(), static_cast<int>(size))) {
            auto received_msg = get_status_msg(packet);
            std::lock_guard<std::mutex> lock(status_mutex_);
            for (const auto & received_status : received_msg.robots_status) {
              if (
                received_status.robot_id < 0 ||
                static_cast<size_t>(received_status.robot_id) >= target.robots_status.size()) {
                continue;
              }
              target.robots_status[received_status.robot_id] = received_status;
            }
          }
        }
      };
    };

    yellow_receiver->startReceive(make_recv_callback(yellow_status_msg));
    blue_receiver->startReceive(make_recv_callback(blue_status_msg));

    asio_ctx_.start();

    pub_robots_status_blue =
      create_publisher<robocup_ssl_msgs::msg::RobotsStatus>("/robots_status/blue", 10);
    pub_robots_status_yellow =
      create_publisher<robocup_ssl_msgs::msg::RobotsStatus>("/robots_status/yellow", 10);

    using std::chrono_literals::operator""ms;
    timer = rclcpp::create_timer(
      this, get_clock(), 10ms, std::bind(&GrSimRobotStatusNode::on_timer, this));
  }

protected:
  auto on_timer() -> void
  {
    auto yellow_msg = std::make_unique<robocup_ssl_msgs::msg::RobotsStatus>();
    auto blue_msg = std::make_unique<robocup_ssl_msgs::msg::RobotsStatus>();
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      *yellow_msg = yellow_status_msg;
      *blue_msg = blue_status_msg;
    }
    pub_robots_status_blue->publish(std::move(blue_msg));
    pub_robots_status_yellow->publish(std::move(yellow_msg));
  }

private:
  auto get_status_msg(const robocup_ssl::Robots_Status & robots_status)
    -> robocup_ssl_msgs::msg::RobotsStatus
  {
    auto statuses_msg = robocup_ssl_msgs::msg::RobotsStatus();

    for (const auto & status : robots_status.robots_status()) {
      if (status.robot_id() < 0 || status.robot_id() >= 20) {
        continue;
      }
      robocup_ssl_msgs::msg::RobotStatus status_msg;
      status_msg.robot_id = status.robot_id();
      status_msg.infrared = status.infrared();
      status_msg.flat_kick = status.flat_kick();
      status_msg.chip_kick = status.chip_kick();

      statuses_msg.robots_status.push_back(std::move(status_msg));
    }

    return statuses_msg;
  }

  crane::AsioContext asio_ctx_;

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<crane::AsyncUdpReceiver> yellow_receiver;
  std::unique_ptr<crane::AsyncUdpReceiver> blue_receiver;

  std::mutex status_mutex_;
  robocup_ssl_msgs::msg::RobotsStatus yellow_status_msg;
  robocup_ssl_msgs::msg::RobotsStatus blue_status_msg;

  rclcpp::Publisher<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr pub_robots_status_blue;
  rclcpp::Publisher<robocup_ssl_msgs::msg::RobotsStatus>::SharedPtr pub_robots_status_yellow;
};
}  // namespace crane

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::GrSimRobotStatusNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
