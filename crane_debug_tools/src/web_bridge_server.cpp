// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <asio.hpp>
#include <cctype>
#include <cmath>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

using json = nlohmann::json;
using WebSocketServer = websocketpp::server<websocketpp::config::asio>;

class WebBridgeServer : public rclcpp::Node
{
public:
  WebBridgeServer() : Node("web_bridge_server"), port_(8080)
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
      "/world_model", 10,
      [this](const crane_msgs::msg::WorldModel::SharedPtr msg) { broadcastWorldModel(msg); });

    robot_commands_sub_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands", 10,
      [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) { broadcastRobotCommands(msg); });

    // Initialize WebSocket server
    initializeWebSocketServer();

    RCLCPP_INFO(this->get_logger(), "Web Bridge Server starting on port %d", port_);
    RCLCPP_INFO(this->get_logger(), "Web root directory: %s", web_root_.c_str());
  }

  void run()
  {
    // Start WebSocket server in separate thread
    websocket_thread_ = std::thread([this]() {
      try {
        server_.listen(port_);
        server_.start_accept();
        server_.run();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "WebSocket server error: %s", e.what());
      }
    });

    // Keep the ROS node spinning
    rclcpp::spin(shared_from_this());
  }

  ~WebBridgeServer()
  {
    server_.stop();
    if (websocket_thread_.joinable()) {
      websocket_thread_.join();
    }
  }

private:
  void initializeWebSocketServer()
  {
    server_.set_access_channels(websocketpp::log::alevel::all);
    server_.clear_access_channels(websocketpp::log::alevel::frame_payload);
    server_.init_asio();

    // Set HTTP handler for serving static files
    server_.set_http_handler([this](websocketpp::connection_hdl hdl) { handleHttpRequest(hdl); });

    server_.set_message_handler(
      [this](websocketpp::connection_hdl hdl, WebSocketServer::message_ptr msg) {
        handleWebSocketMessage(hdl, msg);
      });

    server_.set_open_handler([this](websocketpp::connection_hdl hdl) {
      std::lock_guard<std::mutex> lock(connections_mutex_);
      connections_.insert(hdl);
      RCLCPP_INFO(this->get_logger(), "WebSocket connection opened");
    });

    server_.set_close_handler([this](websocketpp::connection_hdl hdl) {
      std::lock_guard<std::mutex> lock(connections_mutex_);
      connections_.erase(hdl);
      RCLCPP_INFO(this->get_logger(), "WebSocket connection closed");
    });
  }

  void handleHttpRequest(websocketpp::connection_hdl hdl)
  {
    WebSocketServer::connection_ptr con = server_.get_con_from_hdl(hdl);
    std::string uri = con->get_uri()->get_resource();

    // Default to index.html for root request
    if (uri == "/" || uri.empty()) {
      uri = "/index.html";
    }

    std::string file_path = web_root_ + uri;
    std::string content_type = getContentType(uri);

    try {
      std::string file_content = readFile(file_path);

      con->set_status(websocketpp::http::status_code::ok);
      con->set_header("Content-Type", content_type);
      con->set_header("Content-Length", std::to_string(file_content.size()));
      con->set_body(file_content);

      RCLCPP_DEBUG(
        this->get_logger(), "Served file: %s (%s)", file_path.c_str(), content_type.c_str());
    } catch (const std::exception & e) {
      // File not found or error reading
      std::string error_body =
        "<!DOCTYPE html><html><head><title>404 Not Found</title></head>"
        "<body><h1>404 Not Found</h1><p>The requested file was not found.</p></body></html>";

      con->set_status(websocketpp::http::status_code::not_found);
      con->set_header("Content-Type", "text/html");
      con->set_header("Content-Length", std::to_string(error_body.size()));
      con->set_body(error_body);

      RCLCPP_WARN(this->get_logger(), "File not found: %s", file_path.c_str());
    }
  }

  std::string readFile(const std::string & file_path)
  {
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open file: " + file_path);
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
  }

  std::string getContentType(const std::string & uri)
  {
    std::string extension;
    size_t dot_pos = uri.find_last_of('.');
    if (dot_pos != std::string::npos) {
      extension = uri.substr(dot_pos + 1);
    }

    if (extension == "html" || extension == "htm") {
      return "text/html";
    } else if (extension == "css") {
      return "text/css";
    } else if (extension == "js") {
      return "application/javascript";
    } else if (extension == "json") {
      return "application/json";
    } else if (extension == "png") {
      return "image/png";
    } else if (extension == "jpg" || extension == "jpeg") {
      return "image/jpeg";
    } else if (extension == "gif") {
      return "image/gif";
    } else if (extension == "svg") {
      return "image/svg+xml";
    } else {
      return "text/plain";
    }
  }

  void handleWebSocketMessage(websocketpp::connection_hdl hdl, WebSocketServer::message_ptr msg)
  {
    try {
      json request = json::parse(msg->get_payload());
      std::string type = request["type"];

      if (type == "get_world_model") {
        // World model is automatically broadcasted, but we can send current state if needed
        json response = {
          {"type", "world_model_request_acknowledged"},
          {"message", "World model updates are streamed automatically"}};
        sendToConnection(hdl, response);
      } else {
        json error_response = {{"type", "error"}, {"message", "Unknown request type: " + type}};
        sendToConnection(hdl, error_response);
      }
    } catch (const std::exception & e) {
      json error_response = {
        {"type", "error"}, {"message", "Failed to parse request: " + std::string(e.what())}};
      sendToConnection(hdl, error_response);
    }
  }

  void broadcastWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    json world_model = {
      {"type", "world_model"},
      {"timestamp", msg->header.stamp.sec * 1000000000L + msg->header.stamp.nanosec},
      {"is_yellow", msg->is_yellow},
      {"ball",
       {{"x", msg->ball_info.pose.position.x},
        {"y", msg->ball_info.pose.position.y},
        {"z", msg->ball_info.pose.position.z},
        {"vx", msg->ball_info.velocity.x},
        {"vy", msg->ball_info.velocity.y},
        {"vz", msg->ball_info.velocity.z}}},
      {"robots_ours", json::array()},
      {"robots_theirs", json::array()}};

    for (const auto & robot : msg->robot_info_ours) {
      json robot_json = {
        {"id", robot.id},
        {"x", robot.pose.position.x},
        {"y", robot.pose.position.y},
        {"theta", robot.pose.theta},
        {"vx", robot.velocity.x},
        {"vy", robot.velocity.y},
        {"omega", robot.velocity.theta},
        {"team", "ours"}};
      world_model["robots_ours"].push_back(robot_json);
    }

    for (const auto & robot : msg->robot_info_theirs) {
      json robot_json = {
        {"id", robot.id},
        {"x", robot.pose.position.x},
        {"y", robot.pose.position.y},
        {"theta", robot.pose.theta},
        {"vx", robot.velocity.x},
        {"vy", robot.velocity.y},
        {"omega", robot.velocity.theta},
        {"team", "theirs"}};
      world_model["robots_theirs"].push_back(robot_json);
    }

    broadcastToAll(world_model);
  }

  void broadcastRobotCommands(const crane_msgs::msg::RobotCommands::SharedPtr msg)
  {
    json commands = {{"type", "robot_commands"}, {"commands", json::array()}};

    for (const auto & cmd : msg->robot_commands) {
      const bool has_position_target = !cmd.position_target_mode.empty();
      const auto target_x =
        has_position_target ? cmd.position_target_mode.front().target_x : std::nanf("");
      const auto target_y =
        has_position_target ? cmd.position_target_mode.front().target_y : std::nanf("");
      json cmd_json = {{"robot_id", cmd.robot_id},      {"target_x", target_x},
                       {"target_y", target_y},          {"target_theta", cmd.target_theta},
                       {"kick_power", cmd.kick_power},  {"dribble_power", cmd.dribble_power},
                       {"chip_enable", cmd.chip_enable}};
      commands["commands"].push_back(cmd_json);
    }

    broadcastToAll(commands);
  }

  void sendToConnection(websocketpp::connection_hdl hdl, const json & message)
  {
    try {
      server_.get_alog().write(websocketpp::log::alevel::app, message.dump());
      server_.send(hdl, message.dump(), websocketpp::frame::opcode::text);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send message: %s", e.what());
    }
  }

  void broadcastToAll(const json & message)
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto hdl : connections_) {
      sendToConnection(hdl, message);
    }
  }

  // ROS components
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub_;
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr robot_commands_sub_;

  // WebSocket components
  WebSocketServer server_;
  std::thread websocket_thread_;
  std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections_;
  std::mutex connections_mutex_;
  int port_;

  // HTTP components
  std::string package_share_dir_;
  std::string web_root_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WebBridgeServer>();
  node->run();

  rclcpp::shutdown();
  return 0;
}
