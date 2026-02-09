// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cctype>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>

class SimpleWebServer : public rclcpp::Node
{
public:
  SimpleWebServer() : Node("simple_web_server")
  {
    this->declare_parameter("port", 8080);
    port_ = this->get_parameter("port").as_int();

    // Get package share directory for static files
    try {
      package_share_dir_ = ament_index_cpp::get_package_share_directory("crane_debug_tools");
      web_root_ = package_share_dir_ + "/web";
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Could not find package share directory: %s", e.what());
      web_root_ = "./web";  // fallback to local directory
    }

    // Initialize subscribers
    world_model_sub_ = this->create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10, [this](const crane_msgs::msg::WorldModel::SharedPtr msg) {
        // Store latest world model data
        latest_world_model_ = msg;
      });

    robot_commands_sub_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands", 10, [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) {
        // Store latest robot commands
        latest_robot_commands_ = msg;
      });

    // Start HTTP server
    startHttpServer();

    RCLCPP_INFO(this->get_logger(), "Simple Web Server starting on port %d", port_);
    RCLCPP_INFO(this->get_logger(), "Web root directory: %s", web_root_.c_str());
    RCLCPP_INFO(this->get_logger(), "Open http://localhost:%d in your browser", port_);
  }

private:
  void startHttpServer()
  {
    server_thread_ = std::thread([this]() {
      // Simple Python HTTP server for static files
      std::string command =
        "cd " + web_root_ + " && python3 -m http.server " + std::to_string(port_);
      int result = system(command.c_str());
      if (result != 0) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to start HTTP server with command: %s", command.c_str());
      }
    });
  }

  // ROS components
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub_;
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr robot_commands_sub_;

  // HTTP server components
  std::thread server_thread_;
  std::string package_share_dir_;
  std::string web_root_;
  int port_;

  // Latest data storage
  crane_msgs::msg::WorldModel::SharedPtr latest_world_model_;
  crane_msgs::msg::RobotCommands::SharedPtr latest_robot_commands_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SimpleWebServer>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
