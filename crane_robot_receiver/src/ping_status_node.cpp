// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <array>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <cstdio>
#include <format>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

auto getRobotIP(uint8_t id) -> std::string { return std::format("192.168.20.{}", 100 + id); }

class PingNode : public rclcpp::Node
{
public:
  PingNode() : Node("ping_node")
  {
    publisher = this->create_publisher<crane_msgs::msg::PingStatusArray>("/ping", 10);

    for (size_t i = 0; i < robot_ips.size(); ++i) {
      robot_ips[i] = getRobotIP(static_cast<int>(i));
    }

    timer = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PingNode::pingHosts, this));
  }

private:
  auto pingHosts() const -> void
  {
    auto message = crane_msgs::msg::PingStatusArray();
    for (size_t id = 0; id < robot_ips.size(); ++id) {
      const auto & ip = robot_ips[id];
      if (!rclcpp::ok()) {
        return;
      }
      std::string command = "ping -c 1 -W 0.4 " + ip + " | grep 'time='";
      std::array<char, 128> buffer;
      std::string result;

      auto pipe_deleter = [](FILE * pipe_handle) {
        if (pipe_handle != nullptr) {
          (void)pclose(pipe_handle);
        }
      };
      std::unique_ptr<FILE, decltype(pipe_deleter)> pipe(popen(command.c_str(), "r"), pipe_deleter);
      if (!pipe) {
        RCLCPP_ERROR(this->get_logger(), "Failed to run ping command");
        continue;
      }

      while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
      }

      if (not result.empty()) {
        crane_msgs::msg::PingStatus ping_status;
        ping_status.robot_id = static_cast<uint8_t>(id);
        ping_status.ping_ms = std::stod(
          result.substr(result.find("time=") + 5, result.find("ms") - result.find("time=") - 5));
        message.ping.push_back(ping_status);
      }
    }
    publisher->publish(message);
  }

  rclcpp::Publisher<crane_msgs::msg::PingStatusArray>::SharedPtr publisher;

  rclcpp::TimerBase::SharedPtr timer;

  std::array<std::string, 11> robot_ips;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingNode>());
  rclcpp::shutdown();
  return 0;
}
