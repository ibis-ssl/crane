// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <crane_msgs/action/skill_execution.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>
#include <set>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <asio.hpp>

using json = nlohmann::json;

class WebBridgeServer : public rclcpp::Node
{
public:
  using SkillExecutionAction = crane_msgs::action::SkillExecution;
  using SkillExecutionClient = rclcpp_action::Client<SkillExecutionAction>;

  WebBridgeServer() : Node("web_bridge_server"), port_(8080)
  {
    this->declare_parameter("port", 8080);
    port_ = this->get_parameter("port").as_int();

    // Get package share directory for static files
    try {
      package_share_dir_ = ament_index_cpp::get_package_share_directory("crane_debug_tools");
      web_root_ = package_share_dir_ + "/web";
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Could not find package share directory: %s", e.what());
      web_root_ = "./web";  // fallback to local directory
    }

    // Initialize action client
    skill_client_ = rclcpp_action::create_client<SkillExecutionAction>(
      this, "/simple_ai/skill_execution");

    // Initialize subscribers
    world_model_sub_ = this->create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10,
      [this](const crane_msgs::msg::WorldModel::SharedPtr msg) {
        broadcastWorldModel(msg);
      });

    robot_commands_sub_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands", 10,
      [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) {
        broadcastRobotCommands(msg);
      });

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
      } catch (const std::exception& e) {
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
    server_.set_http_handler([this](websocketpp::connection_hdl hdl) {
      handleHttpRequest(hdl);
    });

    server_.set_message_handler([this](websocketpp::connection_hdl hdl, WebSocketServer::message_ptr msg) {
      handleWebSocketMessage(hdl, msg);
    });

    server_.set_open_handler([this](websocketpp::connection_hdl hdl) {
      std::lock_guard<std::mutex> lock(connections_mutex_);
      connections_.insert(hdl);
      RCLCPP_INFO(this->get_logger(), "WebSocket connection opened");
      
      // Send available skills list to new connection
      sendAvailableSkills(hdl);
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
      
      RCLCPP_DEBUG(this->get_logger(), "Served file: %s (%s)", file_path.c_str(), content_type.c_str());
    } catch (const std::exception& e) {
      // File not found or error reading
      std::string error_body = "<!DOCTYPE html><html><head><title>404 Not Found</title></head>"
                              "<body><h1>404 Not Found</h1><p>The requested file was not found.</p></body></html>";
      
      con->set_status(websocketpp::http::status_code::not_found);
      con->set_header("Content-Type", "text/html");
      con->set_header("Content-Length", std::to_string(error_body.size()));
      con->set_body(error_body);
      
      RCLCPP_WARN(this->get_logger(), "File not found: %s", file_path.c_str());
    }
  }

  std::string readFile(const std::string& file_path)
  {
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open file: " + file_path);
    }
    
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
  }

  std::string getContentType(const std::string& uri)
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

  // Parameter type detection functions
  bool isBooleanString(const std::string& value) const
  {
    std::string lower_value = value;
    std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
    return lower_value == "true" || lower_value == "false";
  }

  bool parseBool(const std::string& value) const
  {
    std::string lower_value = value;
    std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
    return lower_value == "true";
  }

  bool isIntegerString(const std::string& value) const
  {
    if (value.empty()) return false;
    
    size_t start = 0;
    if (value[0] == '-' || value[0] == '+') start = 1;
    
    if (start >= value.length()) return false;
    
    for (size_t i = start; i < value.length(); ++i) {
      if (!std::isdigit(value[i])) return false;
    }
    return true;
  }

  bool isFloatString(const std::string& value) const
  {
    if (value.empty()) return false;
    
    try {
      size_t pos;
      std::stof(value, &pos);
      return pos == value.length() && value.find('.') != std::string::npos;
    } catch (...) {
      return false;
    }
  }

  void handleWebSocketMessage(websocketpp::connection_hdl hdl, WebSocketServer::message_ptr msg)
  {
    try {
      json request = json::parse(msg->get_payload());
      std::string type = request["type"];

      if (type == "execute_skill") {
        handleSkillExecution(hdl, request);
      } else if (type == "get_skills") {
        sendAvailableSkills(hdl);
      } else if (type == "get_world_model") {
        // World model is automatically broadcasted, but we can send current state if needed
        json response = {
          {"type", "world_model_request_acknowledged"},
          {"message", "World model updates are streamed automatically"}
        };
        sendToConnection(hdl, response);
      } else {
        json error_response = {
          {"type", "error"},
          {"message", "Unknown request type: " + type}
        };
        sendToConnection(hdl, error_response);
      }
    } catch (const std::exception& e) {
      json error_response = {
        {"type", "error"},
        {"message", "Failed to parse request: " + std::string(e.what())}
      };
      sendToConnection(hdl, error_response);
    }
  }

  void handleSkillExecution(websocketpp::connection_hdl hdl, const json& request)
  {
    try {
      auto goal_msg = SkillExecutionAction::Goal();
      goal_msg.robot_id = request["robot_id"];
      goal_msg.name = request["skill_name"];

      // Parse parameters  
      if (request.contains("parameters")) {
        for (const auto& param : request["parameters"]) {
          std::string name = param["name"];
          std::string value = param["value"];
          
          // Auto-detect parameter type and add to appropriate array
          if (isBooleanString(value)) {
            crane_msgs::msg::NamedBool bool_param;
            bool_param.name = name;
            bool_param.value = parseBool(value);
            goal_msg.parameter.bool_values.push_back(bool_param);
          } else if (isIntegerString(value)) {
            crane_msgs::msg::NamedInt int_param;
            int_param.name = name;
            int_param.value = std::stoi(value);
            goal_msg.parameter.int_values.push_back(int_param);
          } else if (isFloatString(value)) {
            crane_msgs::msg::NamedFloat float_param;
            float_param.name = name;
            float_param.value = std::stof(value);
            goal_msg.parameter.float_values.push_back(float_param);
          } else {
            crane_msgs::msg::NamedString string_param;
            string_param.name = name;
            string_param.value = value;
            goal_msg.parameter.string_values.push_back(string_param);
          }
        }
      }

      // Send goal to action server
      if (!skill_client_->wait_for_action_server(std::chrono::seconds(1))) {
        json error_response = {
          {"type", "error"},
          {"message", "Skill execution action server not available"}
        };
        sendToConnection(hdl, error_response);
        return;
      }

      auto send_goal_options = rclcpp_action::Client<SkillExecutionAction>::SendGoalOptions();
      
      send_goal_options.goal_response_callback =
        [this, hdl](const SkillExecutionClient::GoalHandle::SharedPtr & goal_handle) {
          json response = {
            {"type", "skill_goal_response"},
            {"accepted", goal_handle != nullptr}
          };
          sendToConnection(hdl, response);
        };

      send_goal_options.feedback_callback =
        [this, hdl](const SkillExecutionClient::GoalHandle::SharedPtr &,
                    const std::shared_ptr<const SkillExecutionAction::Feedback> & feedback) {
          json response = {
            {"type", "skill_feedback"},
            {"message", feedback->message}
          };
          sendToConnection(hdl, response);
        };

      send_goal_options.result_callback =
        [this, hdl](const SkillExecutionClient::GoalHandle::WrappedResult & result) {
          json response = {
            {"type", "skill_result"},
            {"code", static_cast<int>(result.code)},
            {"result", result.result ? result.result->result : 0}
          };
          sendToConnection(hdl, response);
        };

      skill_client_->async_send_goal(goal_msg, send_goal_options);

      json ack_response = {
        {"type", "skill_execution_started"},
        {"skill_name", goal_msg.name},
        {"robot_id", goal_msg.robot_id}
      };
      sendToConnection(hdl, ack_response);

    } catch (const std::exception& e) {
      json error_response = {
        {"type", "error"},
        {"message", "Failed to execute skill: " + std::string(e.what())}
      };
      sendToConnection(hdl, error_response);
    }
  }

  void sendAvailableSkills(websocketpp::connection_hdl hdl)
  {
    json skills_list = {
      {"type", "available_skills"},
      {"skills", {
        "Sleep", "Idle", "Kick", "Receive", "Goalie", "Attacker", "SubAttacker",
        "StealBall", "SingleBallPlacement", "GoalKick", "SimpleKickOff", 
        "KickOffAttack", "KickOffSupport", "Marker", "TestMotionPosition", 
        "TestMotionVelocity", "EmplaceRobot", "Forward", "BallNearbyPositioner",
        "GoOverBall", "SecondThreatDefender", "FreekickSaver", "PenaltyKick", "Teleop"
      }}
    };
    sendToConnection(hdl, skills_list);
  }

  void broadcastWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    json world_model = {
      {"type", "world_model"},
      {"timestamp", msg->header.stamp.sec * 1000000000L + msg->header.stamp.nanosec},
      {"is_yellow", msg->is_yellow},
      {"ball", {
        {"x", msg->ball_info.pose.position.x},
        {"y", msg->ball_info.pose.position.y},
        {"z", msg->ball_info.pose.position.z},
        {"vx", msg->ball_info.velocity.x},
        {"vy", msg->ball_info.velocity.y},
        {"vz", msg->ball_info.velocity.z}
      }},
      {"robots_ours", json::array()},
      {"robots_theirs", json::array()}
    };

    for (const auto& robot : msg->robot_info_ours) {
      json robot_json = {
        {"id", robot.id},
        {"x", robot.pose.position.x},
        {"y", robot.pose.position.y},
        {"theta", robot.pose.theta},
        {"vx", robot.velocity.x},
        {"vy", robot.velocity.y},
        {"omega", robot.velocity.theta},
        {"team", "ours"}
      };
      world_model["robots_ours"].push_back(robot_json);
    }

    for (const auto& robot : msg->robot_info_theirs) {
      json robot_json = {
        {"id", robot.id},
        {"x", robot.pose.position.x},
        {"y", robot.pose.position.y},
        {"theta", robot.pose.theta},
        {"vx", robot.velocity.x},
        {"vy", robot.velocity.y},
        {"omega", robot.velocity.theta},
        {"team", "theirs"}
      };
      world_model["robots_theirs"].push_back(robot_json);
    }

    broadcastToAll(world_model);
  }

  void broadcastRobotCommands(const crane_msgs::msg::RobotCommands::SharedPtr msg)
  {
    json commands = {
      {"type", "robot_commands"},
      {"commands", json::array()}
    };

    for (const auto& cmd : msg->robot_commands) {
      json cmd_json = {
        {"robot_id", cmd.robot_id},
        {"target_x", cmd.target_x},
        {"target_y", cmd.target_y},
        {"target_theta", cmd.target_theta},
        {"kick_power", cmd.kick_power},
        {"dribble_power", cmd.dribble_power},
        {"chip_enable", cmd.chip_enable}
      };
      commands["commands"].push_back(cmd_json);
    }

    broadcastToAll(commands);
  }

  void sendToConnection(websocketpp::connection_hdl hdl, const json& message)
  {
    try {
      server_.get_alog().write(websocketpp::log::alevel::app, message.dump());
      server_.send(hdl, message.dump(), websocketpp::frame::opcode::text);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send message: %s", e.what());
    }
  }

  void broadcastToAll(const json& message)
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto hdl : connections_) {
      sendToConnection(hdl, message);
    }
  }

  // ROS components
  SkillExecutionClient::SharedPtr skill_client_;
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