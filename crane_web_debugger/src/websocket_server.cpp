// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <openssl/evp.h>
#include <openssl/sha.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/asio.hpp>
#include <cctype>
#include <chrono>
#include <crane_msgs/msg/human_annotation.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/position_target_mode.hpp>
#include <crane_msgs/msg/robot_command.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_visualization_interfaces/msg/svg_snapshot.hpp>
#include <crane_visualization_interfaces/msg/svg_updates.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <thread>

using json = nlohmann::json;

// Simple WebSocket frame implementation
struct WebSocketFrame
{
  bool fin;
  uint8_t opcode;
  bool masked;
  uint64_t payload_length;
  uint32_t masking_key;
  std::string payload;
};

class WebSocketConnection
{
public:
  explicit WebSocketConnection(std::shared_ptr<boost::asio::ip::tcp::socket> socket)
  : socket_(socket), connected_(false)
  {
  }

  bool handshake()
  {
    try {
      // Read HTTP request
      boost::asio::streambuf buffer;
      boost::asio::read_until(*socket_, buffer, "\r\n\r\n");

      std::istream request_stream(&buffer);
      std::string line;
      std::string websocket_key;

      // Parse HTTP headers
      while (std::getline(request_stream, line) && line != "\r") {
        if (line.starts_with("Sec-WebSocket-Key:")) {
          websocket_key = line.substr(19);
          // Remove leading/trailing whitespace
          websocket_key.erase(0, websocket_key.find_first_not_of(" \t\r\n"));
          websocket_key.erase(websocket_key.find_last_not_of(" \t\r\n") + 1);
        }
      }

      if (websocket_key.empty()) {
        return false;
      }

      // Generate WebSocket accept key
      std::string accept_key = generateAcceptKey(websocket_key);

      // Send WebSocket handshake response
      std::string response =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " +
        accept_key +
        "\r\n"
        "\r\n";

      boost::asio::write(*socket_, boost::asio::buffer(response));
      connected_ = true;
      return true;
    } catch (const std::exception & e) {
      return false;
    }
  }

  void sendMessage(const std::string & message)
  {
    if (!connected_) return;

    try {
      std::vector<uint8_t> frame;

      // FIN=1, opcode=1 (text frame)
      frame.push_back(0x81);

      // Payload length
      if (message.length() < 126) {
        frame.push_back(static_cast<uint8_t>(message.length()));
      } else if (message.length() < 65536) {
        frame.push_back(126);
        frame.push_back((message.length() >> 8) & 0xFF);
        frame.push_back(message.length() & 0xFF);
      } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; i--) {
          frame.push_back((message.length() >> (i * 8)) & 0xFF);
        }
      }

      // Payload
      frame.insert(frame.end(), message.begin(), message.end());

      boost::asio::write(*socket_, boost::asio::buffer(frame));
    } catch (const std::exception & e) {
      connected_ = false;
    }
  }

  std::string receiveMessage()
  {
    if (!connected_) return "";

    try {
      uint8_t header[2];
      boost::asio::read(*socket_, boost::asio::buffer(header, 2));

      // bool fin = (header[0] & 0x80) != 0;  // unused
      // uint8_t opcode = header[0] & 0x0F;   // unused
      bool masked = (header[1] & 0x80) != 0;
      uint64_t payload_length = header[1] & 0x7F;

      // Read extended payload length if necessary
      if (payload_length == 126) {
        uint8_t extended[2];
        boost::asio::read(*socket_, boost::asio::buffer(extended, 2));
        payload_length = (extended[0] << 8) | extended[1];
      } else if (payload_length == 127) {
        uint8_t extended[8];
        boost::asio::read(*socket_, boost::asio::buffer(extended, 8));
        payload_length = 0;
        for (int i = 0; i < 8; i++) {
          payload_length = (payload_length << 8) | extended[i];
        }
      }

      // Read masking key if present
      uint32_t masking_key = 0;
      if (masked) {
        uint8_t mask[4];
        boost::asio::read(*socket_, boost::asio::buffer(mask, 4));
        masking_key = (mask[0] << 24) | (mask[1] << 16) | (mask[2] << 8) | mask[3];
      }

      // Read payload
      std::vector<uint8_t> payload(payload_length);
      if (payload_length > 0) {
        boost::asio::read(*socket_, boost::asio::buffer(payload));

        // Unmask payload if necessary
        if (masked) {
          for (uint64_t i = 0; i < payload_length; i++) {
            payload[i] ^= ((masking_key >> (24 - (i % 4) * 8)) & 0xFF);
          }
        }
      }

      return std::string(payload.begin(), payload.end());
    } catch (const std::exception & e) {
      connected_ = false;
      return "";
    }
  }

  bool isConnected() const { return connected_; }

  void close()
  {
    connected_ = false;
    if (socket_) {
      socket_->close();
    }
  }

private:
  std::string generateAcceptKey(const std::string & key)
  {
    // WebSocket GUID as per RFC 6455
    const std::string websocket_magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    std::string concat = key + websocket_magic;

    // Calculate SHA-1 hash
    unsigned char hash[SHA_DIGEST_LENGTH];
    SHA1(reinterpret_cast<const unsigned char *>(concat.c_str()), concat.length(), hash);

    // Base64 encode
    BIO *bmem, *b64;
    BUF_MEM * bptr;

    b64 = BIO_new(BIO_f_base64());
    bmem = BIO_new(BIO_s_mem());
    b64 = BIO_push(b64, bmem);
    BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);
    BIO_write(b64, hash, SHA_DIGEST_LENGTH);
    BIO_flush(b64);
    BIO_get_mem_ptr(b64, &bptr);

    std::string result(bptr->data, bptr->length);
    BIO_free_all(b64);

    return result;
  }

  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  bool connected_;
};

class WebSocketDebugServer : public rclcpp::Node
{
public:
  WebSocketDebugServer() : Node("websocket_debug_server"), port_(8090), websocket_port_(8091)
  {
    this->declare_parameter("port", 8090);
    this->declare_parameter("websocket_port", 8091);
    port_ = this->get_parameter("port").as_int();
    websocket_port_ = this->get_parameter("websocket_port").as_int();

    // Get package share directory for static files
    try {
      package_share_dir_ = ament_index_cpp::get_package_share_directory("crane_web_debugger");
      web_root_ = package_share_dir_ + "/web";
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Could not find package share directory: %s", e.what());
      web_root_ = "./web";
    }

    // Initialize subscribers
    world_model_sub_ = this->create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10, [this](const crane_msgs::msg::WorldModel::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(world_model_throttle_mutex_);
        latest_world_model_ = msg;
        world_model_updated_ = true;
      });

    robot_commands_sub_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands", 10,
      [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) { broadcastRobotCommands(msg); });

    play_situation_sub_ = this->create_subscription<crane_msgs::msg::PlaySituation>(
      "/play_situation", 10, [this](const crane_msgs::msg::PlaySituation::SharedPtr msg) {
        // 最新のメッセージをキャッシュ
        {
          std::lock_guard<std::mutex> lock(game_info_mutex_);
          latest_play_situation_ = msg;
        }
        broadcastGameInfo(msg);
      });

    aggregated_svgs_sub_ =
      this->create_subscription<crane_visualization_interfaces::msg::SvgSnapshot>(
        "/aggregated_svgs", 10,
        [this](const crane_visualization_interfaces::msg::SvgSnapshot::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(svg_snapshot_mutex_);
          latest_svg_snapshot_ = msg;
          svg_snapshot_updated_ = true;
        });

    // High-frequency incremental SVG updates — accumulate, flush at 20Hz
    visualizer_svgs_sub_ =
      this->create_subscription<crane_visualization_interfaces::msg::SvgUpdates>(
        "/visualizer_svgs", rclcpp::SensorDataQoS(),
        [this](const crane_visualization_interfaces::msg::SvgUpdates::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(svg_updates_mutex_);
          pending_svg_updates_.push_back(msg);
        });

    // Human annotation publisher
    annotation_pub_ =
      this->create_publisher<crane_msgs::msg::HumanAnnotation>("/human_annotations", 10);

    // Robot move command publishers
    move_command_pub_ =
      this->create_publisher<crane_msgs::msg::RobotCommands>("/control_targets", 10);
    session_injection_pub_ =
      this->create_publisher<std_msgs::msg::String>("/session_injection", 10);

    // Robot feedback subscription (cached, broadcast at 10Hz via timer)
    robot_feedback_sub_ = this->create_subscription<crane_msgs::msg::RobotFeedbackArray>(
      "/robot_feedback", 10, [this](const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(robot_feedback_mutex_);
        latest_robot_feedback_ = msg;
        robot_feedback_updated_ = true;
      });

    ping_sub_ = this->create_subscription<crane_msgs::msg::PingStatusArray>(
      "/ping", 10,
      [this](const crane_msgs::msg::PingStatusArray::SharedPtr msg) { broadcastPingStatus(msg); });

    diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics_agg", 10, [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        broadcastDiagnostics(msg);
      });

    // 100ms timer for robot_feedback broadcast (10Hz throttle)
    robot_feedback_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      crane_msgs::msg::RobotFeedbackArray::SharedPtr msg;
      {
        std::lock_guard<std::mutex> lock(robot_feedback_mutex_);
        if (!robot_feedback_updated_ || !latest_robot_feedback_) return;
        msg = latest_robot_feedback_;
        robot_feedback_updated_ = false;
      }
      broadcastRobotFeedback(msg);
    });

    // control_targets subscription (cached, broadcast at 10Hz via timer)
    control_targets_sub_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/control_targets", 10, [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(control_targets_mutex_);
        latest_control_targets_ = msg;
        control_targets_updated_ = true;
      });

    // 100ms timer for control_targets broadcast (10Hz throttle)
    control_targets_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      crane_msgs::msg::RobotCommands::SharedPtr msg;
      {
        std::lock_guard<std::mutex> lock(control_targets_mutex_);
        if (!control_targets_updated_ || !latest_control_targets_) return;
        msg = latest_control_targets_;
        control_targets_updated_ = false;
      }
      broadcastControlTargets(msg);
    });

    // 100ms timer for world_model broadcast (10Hz throttle)
    world_model_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      crane_msgs::msg::WorldModel::SharedPtr msg;
      {
        std::lock_guard<std::mutex> lock(world_model_throttle_mutex_);
        if (!world_model_updated_ || !latest_world_model_) return;
        msg = latest_world_model_;
        world_model_updated_ = false;
      }
      broadcastWorldModel(msg);
    });

    // 200ms timer for aggregated_svgs broadcast (5Hz throttle)
    svg_snapshot_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), [this]() {
      crane_visualization_interfaces::msg::SvgSnapshot::SharedPtr msg;
      {
        std::lock_guard<std::mutex> lock(svg_snapshot_mutex_);
        if (!svg_snapshot_updated_ || !latest_svg_snapshot_) return;
        msg = latest_svg_snapshot_;
        svg_snapshot_updated_ = false;
      }
      broadcastSvgData(msg);
    });

    // 50ms timer for visualizer_svgs broadcast (20Hz coalescing)
    svg_updates_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
      std::vector<crane_visualization_interfaces::msg::SvgUpdates::SharedPtr> batch;
      {
        std::lock_guard<std::mutex> lock(svg_updates_mutex_);
        if (pending_svg_updates_.empty()) return;
        batch.swap(pending_svg_updates_);
      }
      broadcastCoalescedSvgUpdates(batch);
    });

    // 5s timer for Pi status polling
    pi_status_timer_ =
      this->create_wall_timer(std::chrono::seconds(5), [this]() { pollAllPiStatus(); });

    // Start servers
    startServers();

    RCLCPP_INFO(this->get_logger(), "WebSocket Debug Server starting on port %d", port_);
    RCLCPP_INFO(this->get_logger(), "Web root directory: %s", web_root_.c_str());
    RCLCPP_INFO(this->get_logger(), "HTTP: http://localhost:%d", port_);
    RCLCPP_INFO(this->get_logger(), "WebSocket: ws://localhost:%d", websocket_port_);
  }

  ~WebSocketDebugServer() { stopServers(); }

private:
  void startServers()
  {
    // Start HTTP server in separate thread
    http_thread_ = std::thread([this]() { this->runHttpServer(); });

    // Start WebSocket server in separate thread
    websocket_thread_ = std::thread([this]() { this->runWebSocketServer(); });
  }

  void stopServers()
  {
    running_ = false;

    if (http_thread_.joinable()) {
      http_thread_.join();
    }
    if (websocket_thread_.joinable()) {
      websocket_thread_.join();
    }
  }

  void runHttpServer()
  {
    // Check if web directory exists
    if (!std::filesystem::exists(web_root_)) {
      RCLCPP_ERROR(this->get_logger(), "Web directory not found: %s", web_root_.c_str());
      return;
    }

    // Simple Python HTTP server for static files
    // Bind to 0.0.0.0 to accept connections from any interface
    std::string command = "cd \"" + web_root_ + "\" && python3 -m http.server " +
                          std::to_string(port_) + " --bind 0.0.0.0 2>/dev/null";
    RCLCPP_INFO(this->get_logger(), "Starting HTTP server at: %s", web_root_.c_str());
    int result = system(command.c_str());
    if (result != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start HTTP server, exit code: %d", result);
    }
  }

  void runWebSocketServer()
  {
    try {
      boost::asio::io_context io_context;
      boost::asio::ip::tcp::acceptor acceptor(
        io_context, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), websocket_port_));

      RCLCPP_INFO(this->get_logger(), "WebSocket server listening on port %d", websocket_port_);

      while (running_) {
        auto socket = std::make_shared<boost::asio::ip::tcp::socket>(io_context);
        acceptor.accept(*socket);

        // Handle WebSocket connection in separate thread
        std::thread([this, socket]() { handleWebSocketConnection(socket); }).detach();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "WebSocket server error: %s", e.what());
    }
  }

  void handleWebSocketConnection(std::shared_ptr<boost::asio::ip::tcp::socket> socket)
  {
    auto connection = std::make_shared<WebSocketConnection>(socket);

    if (!connection->handshake()) {
      RCLCPP_WARN(this->get_logger(), "WebSocket handshake failed");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(connections_mutex_);
      connections_.insert(connection);
    }

    RCLCPP_INFO(this->get_logger(), "WebSocket connection established");

    // 接続確立時に最新のゲーム情報を送信
    {
      std::lock_guard<std::mutex> lock(game_info_mutex_);
      if (latest_play_situation_) {
        sendGameInfoToConnection(connection, latest_play_situation_);
      }
    }

    // 接続確立時に最新のロボットフィードバックを送信
    {
      crane_msgs::msg::RobotFeedbackArray::SharedPtr fb_msg;
      {
        std::lock_guard<std::mutex> lock(robot_feedback_mutex_);
        fb_msg = latest_robot_feedback_;
      }
      if (fb_msg) {
        broadcastRobotFeedback(fb_msg);
      }
    }

    // 接続確立時にPiステータスを取得・送信
    pollAllPiStatus();

    // Message processing loop
    while (connection->isConnected() && running_) {
      std::string message = connection->receiveMessage();
      if (!message.empty()) {
        handleWebSocketMessage(connection, message);
      } else {
        // Connection closed or error
        break;
      }
    }

    // Clean up connection
    {
      std::lock_guard<std::mutex> lock(connections_mutex_);
      connections_.erase(connection);
    }

    RCLCPP_INFO(this->get_logger(), "WebSocket connection closed");
  }

  void handleWebSocketMessage(
    std::shared_ptr<WebSocketConnection> connection, const std::string & message)
  {
    try {
      json request = json::parse(message);
      std::string type = request["type"];

      if (type == "time_sync_request") {
        handleTimeSyncRequest(connection, request);
      } else if (type == "annotation") {
        handleAnnotation(connection, request);
      } else if (type == "robot_control") {
        handleRobotControl(connection, request);
      } else if (type == "poll_pi_status") {
        pollAllPiStatus();
      } else if (type == "activate_move_mode") {
        handleActivateMoveMode(connection);
      } else if (type == "move_robot") {
        handleMoveRobot(connection, request);
      } else {
        json error_response = {{"type", "error"}, {"message", "Unknown request type: " + type}};
        connection->sendMessage(error_response.dump());
      }
    } catch (const std::exception & e) {
      json error_response = {
        {"type", "error"}, {"message", "Failed to parse request: " + std::string(e.what())}};
      connection->sendMessage(error_response.dump());
    }
  }

  template <typename Checkpoints>
  static json to_delay_checkpoints_json(const Checkpoints & checkpoints)
  {
    json result = json::array();
    for (const auto & checkpoint : checkpoints) {
      result.push_back(
        {{"name", checkpoint.name},
         {"relative_time_us", checkpoint.relative_time_us},
         {"value", checkpoint.value}});
    }
    return result;
  }

  template <typename Checkpoints>
  static json compute_delay_analysis_json(const Checkpoints & checkpoints)
  {
    json delay_analysis = {{"total_delay_ms", 0.0}, {"stage_delays", json::array()}};

    for (size_t i = 1; i < checkpoints.size(); ++i) {
      auto delay_us = checkpoints[i].relative_time_us - checkpoints[i - 1].relative_time_us;
      auto delay_ms = static_cast<double>(delay_us) / 1000.0;

      delay_analysis["stage_delays"].push_back(
        {{"from", checkpoints[i - 1].name}, {"to", checkpoints[i].name}, {"delay_ms", delay_ms}});
    }

    if (checkpoints.size() > 1) {
      auto total_delay_us = checkpoints.back().relative_time_us;
      delay_analysis["total_delay_ms"] = static_cast<double>(total_delay_us) / 1000.0;
    }

    return delay_analysis;
  }

  void broadcastWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(world_model_cache_mutex_);
      cached_is_yellow_ = msg->is_yellow;
      cached_on_positive_half_ = msg->on_positive_half;
    }

    json world_model = {
      {"type", "world_model"},
      {"timestamp", msg->header.stamp.sec * 1000000000L + msg->header.stamp.nanosec},
      {"is_yellow", msg->is_yellow},
      {"on_positive_half", msg->on_positive_half},
      {"ball",
       {{"x", msg->ball_info.position.x},
        {"y", msg->ball_info.position.y},
        {"z", msg->ball_info.position.z},
        {"vx", msg->ball_info.velocity.x},
        {"vy", msg->ball_info.velocity.y},
        {"vz", msg->ball_info.velocity.z}}},
      {"robots_ours", json::array()},
      {"robots_theirs", json::array()}};

    for (const auto & robot : msg->robot_info_ours) {
      json robot_json = {
        {"id", robot.id},
        {"x", robot.pose.x},
        {"y", robot.pose.y},
        {"theta", robot.pose.theta},
        {"vx", robot.velocity.x},
        {"vy", robot.velocity.y},
        {"omega", robot.velocity.theta},
        {"vision_x", robot.vision.pose.x},
        {"vision_y", robot.vision.pose.y},
        {"vision_theta", robot.vision.pose.theta},
        {"vision_stamp_sec", robot.vision.stamp.sec},
        {"vision_stamp_nanosec", robot.vision.stamp.nanosec},
        {"available_vision", robot.available_vision},
        {"available_feedback", robot.available_feedback},
        {"available_tracker", robot.available_tracker},
        {"acceleration_x", robot.acceleration.x},
        {"acceleration_y", robot.acceleration.y},
        {"acceleration_theta", robot.acceleration.theta},
        {"team", "ours"}};
      world_model["robots_ours"].push_back(robot_json);
    }

    for (const auto & robot : msg->robot_info_theirs) {
      json robot_json = {
        {"id", robot.id},
        {"x", robot.pose.x},
        {"y", robot.pose.y},
        {"theta", robot.pose.theta},
        {"vx", robot.velocity.x},
        {"vy", robot.velocity.y},
        {"omega", robot.velocity.theta},
        {"team", "theirs"}};
      world_model["robots_theirs"].push_back(robot_json);
    }

    // WorldModelの遅延監視情報を追加
    world_model["delay_checkpoints"] =
      to_delay_checkpoints_json(msg->delay_checkpoints.checkpoints);
    world_model["delay_reference_timestamp_ns"] = msg->delay_checkpoints.reference_timestamp_ns;

    // WorldModel遅延分析情報
    if (!msg->delay_checkpoints.checkpoints.empty()) {
      world_model["delay_analysis"] =
        compute_delay_analysis_json(msg->delay_checkpoints.checkpoints);
    }

    broadcastToAll(world_model.dump());
  }

  static json robotCommandToJson(const crane_msgs::msg::RobotCommand & cmd)
  {
    json planning_factors_json = json::array();
    for (const auto & factor : cmd.planning_factors) {
      planning_factors_json.push_back({{"name", factor.name}, {"state", factor.value}});
    }

    json position_target_json = nullptr;
    if (!cmd.position_target_mode.empty()) {
      const auto & pt = cmd.position_target_mode[0];
      position_target_json = {
        {"target_x", pt.target_x},
        {"target_y", pt.target_y},
        {"position_tolerance", pt.position_tolerance},
        {"speed_limit_at_target", pt.speed_limit_at_target}};
    }

    json simple_velocity_json = nullptr;
    if (!cmd.simple_velocity_target_mode.empty()) {
      const auto & sv = cmd.simple_velocity_target_mode[0];
      simple_velocity_json = {
        {"target_vx", sv.target_vx},
        {"target_vy", sv.target_vy},
        {"speed_limit_at_target", sv.speed_limit_at_target}};
    }

    json polar_velocity_json = nullptr;
    if (!cmd.polar_velocity_target_mode.empty()) {
      const auto & pv = cmd.polar_velocity_target_mode[0];
      polar_velocity_json = {
        {"target_velocity_r", pv.target_velocity_r},
        {"target_velocity_theta", pv.target_velocity_theta}};
    }

    json local_camera_json = nullptr;
    if (!cmd.local_camera_mode.empty()) {
      const auto & lc = cmd.local_camera_mode[0];
      local_camera_json = {
        {"ball_x", lc.ball_x},
        {"ball_y", lc.ball_y},
        {"ball_vx", lc.ball_vx},
        {"ball_vy", lc.ball_vy},
        {"target_global_vx", lc.target_global_vx},
        {"target_global_vy", lc.target_global_vy}};
    }

    json velocity_plan_trace_json = nullptr;
    if (!cmd.velocity_plan_trace.empty()) {
      const auto & trace = cmd.velocity_plan_trace[0];
      json plan_points_json = json::array();
      for (const auto & pt : trace.plan_points) {
        plan_points_json.push_back(
          {{"source", pt.source},
           {"target_time_us", pt.target_time_us},
           {"predicted_pos_x", pt.predicted_pos_x},
           {"predicted_pos_y", pt.predicted_pos_y},
           {"predicted_vel_x", pt.predicted_vel_x},
           {"predicted_vel_y", pt.predicted_vel_y},
           {"estimated_arrival_time_us", pt.estimated_arrival_time_us}});
      }
      json corrections_json = json::array();
      for (const auto & corr : trace.corrections) {
        corrections_json.push_back(
          {{"source", corr.source},
           {"before_vel_x", corr.before_vel_x},
           {"before_vel_y", corr.before_vel_y},
           {"after_vel_x", corr.after_vel_x},
           {"after_vel_y", corr.after_vel_y},
           {"velocity_delta", corr.velocity_delta},
           {"direction_delta_deg", corr.direction_delta_deg}});
      }
      json actuals_json = json::array();
      for (const auto & actual : trace.actuals) {
        actuals_json.push_back(
          {{"plan_time_us", actual.plan_time_us},
           {"planned_vel_x", actual.planned_vel_x},
           {"planned_vel_y", actual.planned_vel_y},
           {"planned_pos_x", actual.planned_pos_x},
           {"planned_pos_y", actual.planned_pos_y},
           {"actual_vel_x", actual.actual_vel_x},
           {"actual_vel_y", actual.actual_vel_y},
           {"actual_pos_x", actual.actual_pos_x},
           {"actual_pos_y", actual.actual_pos_y},
           {"velocity_error", actual.velocity_error},
           {"position_error", actual.position_error}});
      }
      velocity_plan_trace_json = {
        {"trace_id", trace.trace_id},
        {"reference_timestamp_ns", trace.reference_timestamp_ns},
        {"plan_points", plan_points_json},
        {"corrections", corrections_json},
        {"actuals", actuals_json}};
    }

    return {
      {"robot_id", cmd.robot_id},
      {"kick_power", cmd.kick_power},
      {"dribble_power", cmd.dribble_power},
      {"chip_enable", cmd.chip_enable},
      {"target_theta", cmd.target_theta},
      {"control_mode", cmd.control_mode},
      {"current_pose",
       {{"x", cmd.current_pose.x}, {"y", cmd.current_pose.y}, {"theta", cmd.current_pose.theta}}},
      {"current_velocity",
       {{"vx", cmd.current_velocity.x},
        {"vy", cmd.current_velocity.y},
        {"omega", cmd.current_velocity.theta}}},
      {"planning_factors", planning_factors_json},
      {"planner_name", cmd.planner_name},
      {"delay_checkpoints", to_delay_checkpoints_json(cmd.delay_checkpoints.checkpoints)},
      {"position_target_mode", position_target_json},
      {"simple_velocity_target_mode", simple_velocity_json},
      {"polar_velocity_target_mode", polar_velocity_json},
      {"local_camera_mode", local_camera_json},
      {"velocity_plan_trace", velocity_plan_trace_json}};
  }

  void broadcastRobotCommands(const crane_msgs::msg::RobotCommands::SharedPtr msg)
  {
    json commands = {{"type", "robot_commands"}, {"commands", json::array()}};

    commands["delay_checkpoints"] = to_delay_checkpoints_json(msg->delay_checkpoints.checkpoints);
    commands["delay_reference_timestamp_ns"] = msg->delay_checkpoints.reference_timestamp_ns;

    if (!msg->delay_checkpoints.checkpoints.empty()) {
      commands["delay_analysis"] = compute_delay_analysis_json(msg->delay_checkpoints.checkpoints);
    }

    for (const auto & cmd : msg->robot_commands) {
      commands["commands"].push_back(robotCommandToJson(cmd));
    }

    broadcastToAll(commands.dump());
  }

  void broadcastControlTargets(const crane_msgs::msg::RobotCommands::SharedPtr msg)
  {
    json commands = {{"type", "control_targets"}, {"commands", json::array()}};
    for (const auto & cmd : msg->robot_commands) {
      commands["commands"].push_back(robotCommandToJson(cmd));
    }
    broadcastToAll(commands.dump());
  }

  void broadcastSvgData(const crane_visualization_interfaces::msg::SvgSnapshot::SharedPtr msg)
  {
    const auto stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    json svg_data = {
      {"type", "svg_data"},
      {"epoch", msg->epoch},
      {"seq", msg->seq},
      {"stamp_ns", stamp_ns},
      {"layers", json::array()}};

    for (const auto & layer : msg->layers) {
      json layer_json = {{"layer", layer.layer}, {"svg_primitives", json::array()}};

      for (const auto & primitive : layer.svg_primitives) {
        layer_json["svg_primitives"].push_back(primitive);
      }

      svg_data["layers"].push_back(layer_json);
    }

    broadcastToAll(svg_data.dump());
  }

  void broadcastCoalescedSvgUpdates(
    const std::vector<crane_visualization_interfaces::msg::SvgUpdates::SharedPtr> & batch)
  {
    json svg_update = {{"type", "svg_update"}, {"updates", json::array()}};
    for (const auto & msg : batch) {
      for (const auto & upd : msg->updates) {
        json upd_json = {
          {"layer", upd.layer}, {"operation", upd.operation}, {"svg_primitives", json::array()}};
        for (const auto & primitive : upd.svg_primitives) {
          upd_json["svg_primitives"].push_back(primitive);
        }
        svg_update["updates"].push_back(upd_json);
      }
    }
    broadcastToAll(svg_update.dump());
  }

  void handleTimeSyncRequest(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    // NTPライクな時刻同期
    // T1: クライアント送信時刻
    // T2: サーバー受信時刻（now）
    auto T2 = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();

    json response = {
      {"type", "time_sync_response"},
      {"client_send_time_ms", request["client_send_time_ms"]},
      {"server_receive_time_ms", T2},
      {"server_send_time_ms", T2},  // 処理時間が短いため同じ
      {"ros_time_ns", this->get_clock()->now().nanoseconds()}};

    connection->sendMessage(response.dump());
  }

  void handleAnnotation(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    auto msg = crane_msgs::msg::HumanAnnotation();

    // ヘッダー設定
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "human_annotation";

    // 基本フィールド
    msg.category = request.value("category", 0);
    msg.priority = request.value("priority", 1);
    msg.label = request.value("label", "");
    msg.description = request.value("description", "");

    // 時刻関連
    msg.event_timestamp_ns = request.value("event_timestamp_ns", 0L);
    msg.client_timestamp_ms = request.value("client_timestamp_ms", 0L);
    msg.time_offset_ms = request.value("time_offset_ms", 0);

    // デフォルトのタイムスタンプ
    int64_t default_timestamp = this->get_clock()->now().nanoseconds();
    if (msg.event_timestamp_ns == 0) {
      msg.event_timestamp_ns = default_timestamp;
    }

    // 位置情報
    if (request.contains("position")) {
      msg.has_position = true;
      msg.position.x = request["position"].value("x", 0.0);
      msg.position.y = request["position"].value("y", 0.0);
      msg.position.z = request["position"].value("z", 0.0);
    } else {
      msg.has_position = false;
    }

    // ロボットコンテキスト
    if (request.contains("related_robot_ids")) {
      msg.has_robot_context = true;
      for (const auto & id : request["related_robot_ids"]) {
        msg.related_robot_ids.push_back(id.get<uint8_t>());
      }
      if (request.contains("robot_is_ours")) {
        for (const auto & is_ours : request["robot_is_ours"]) {
          msg.robot_is_ours.push_back(is_ours.get<bool>());
        }
      }
    } else {
      msg.has_robot_context = false;
    }

    // メタデータ
    if (request.contains("metadata")) {
      msg.metadata_json = request["metadata"].dump();
    }

    // パブリッシュ
    annotation_pub_->publish(msg);

    // 確認レスポンス
    json response = {
      {"type", "annotation_ack"}, {"success", true}, {"timestamp_ns", default_timestamp}};
    connection->sendMessage(response.dump());

    RCLCPP_INFO(
      this->get_logger(), "Human annotation received: [%s] %s", msg.label.c_str(),
      msg.description.c_str());
  }

  std::string createGameInfoMessage(const crane_msgs::msg::PlaySituation::SharedPtr msg)
  {
    json game_info = {
      {"type", "game_info"},
      {"play_situation", msg->command.name},
      {"our_score", msg->our_team_info.score},
      {"their_score", msg->their_team_info.score},
      {"game_time", msg->referee_raw.stage_time_left / 1000000},  // マイクロ秒→秒
      {"game_stage", msg->stage.name},
      {"game_event", msg->reason_text}};

    return game_info.dump();
  }

  void sendGameInfoToConnection(
    std::shared_ptr<WebSocketConnection> connection,
    const crane_msgs::msg::PlaySituation::SharedPtr msg)
  {
    if (connection->isConnected()) {
      connection->sendMessage(createGameInfoMessage(msg));
    }
  }

  void broadcastGameInfo(const crane_msgs::msg::PlaySituation::SharedPtr msg)
  {
    broadcastToAll(createGameInfoMessage(msg));
  }

  static std::string getRobotIp(int robot_id)
  {
    return "192.168.20." + std::to_string(100 + robot_id);
  }

  // レスポンスボディから "status" フィールドを抽出する。失敗時は default_failure を返す
  static std::string parseStatusFromBody(
    bool success, const std::string & body, const std::string & default_ok = "OK",
    const std::string & default_failure = "Offline")
  {
    if (!success) return default_failure;
    if (!body.empty()) {
      try {
        auto body_json = json::parse(body);
        if (body_json.contains("status")) {
          return body_json["status"].get<std::string>();
        }
      } catch (...) {
      }
    }
    return default_ok;
  }

  void broadcastRobotFeedback(const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg)
  {
    json data = {{"type", "robot_feedback"}, {"robots", json::array()}};
    for (const auto & robot : msg->feedback) {
      json robot_json = {
        {"robot_id", robot.robot_id},
        {"voltage", robot.voltage},
        {"temperatures", robot.temperatures},
        {"error_id", robot.error_id},
        {"error_info", robot.error_info},
        {"error_value", robot.error_value},
        {"motor_current", robot.motor_current},
        {"ball_sensor", robot.ball_sensor},
        {"kick_state", robot.kick_state},
        {"packet_frequency_hz", robot.packet_frequency_hz},
        {"yaw_angle", robot.yaw_angle},
        {"diff_angle", robot.diff_angle},
        {"odom_speed", robot.odom_speed},
        {"odom", robot.odom},
        {"mouse_vel", robot.mouse_vel},
        {"mouse_odom", robot.mouse_odom},
        {"values", robot.values}};
      data["robots"].push_back(robot_json);
    }
    broadcastToAll(data.dump());
  }

  void broadcastPingStatus(const crane_msgs::msg::PingStatusArray::SharedPtr msg)
  {
    json data = {{"type", "ping_status"}, {"robots", json::array()}};
    for (const auto & ping : msg->ping) {
      data["robots"].push_back({{"robot_id", ping.robot_id}, {"ping_ms", ping.ping_ms}});
    }
    broadcastToAll(data.dump());
  }

  void broadcastDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    json data = {{"type", "diagnostics"}, {"statuses", json::array()}};
    for (const auto & status : msg->status) {
      json values = json::array();
      for (const auto & kv : status.values) {
        values.push_back({{"key", kv.key}, {"value", kv.value}});
      }
      data["statuses"].push_back(
        {{"name", status.name},
         {"level", status.level},
         {"message", status.message},
         {"values", values}});
    }
    broadcastToAll(data.dump());
  }

  // Simple blocking HTTP client with connect/read/write timeouts
  static std::pair<bool, std::string> sendHttpRequest(
    const std::string & host, int port, const std::string & method, const std::string & path,
    int timeout_ms = 500)
  {
    try {
      boost::asio::io_context io_context;
      boost::asio::ip::tcp::resolver resolver(io_context);
      boost::asio::ip::tcp::socket socket(io_context);
      boost::asio::steady_timer timer(io_context);

      boost::system::error_code ec;
      auto endpoints = resolver.resolve(host, std::to_string(port), ec);
      if (ec) return {false, ""};

      // Async connect with timeout
      bool connected = false;
      timer.expires_after(std::chrono::milliseconds(timeout_ms));
      timer.async_wait([&socket](const boost::system::error_code & timer_ec) {
        if (!timer_ec) socket.cancel();
      });
      boost::asio::async_connect(
        socket, endpoints,
        [&](const boost::system::error_code & conn_ec, const boost::asio::ip::tcp::endpoint &) {
          if (!conn_ec) connected = true;
          timer.cancel();
        });
      io_context.run();
      if (!connected) return {false, ""};

      // Set socket read/write timeouts
      timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = static_cast<suseconds_t>(timeout_ms) * 1000;
      setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
      setsockopt(socket.native_handle(), SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

      // Send HTTP request
      std::string req = method + " " + path + " HTTP/1.0\r\n" + "Host: " + host + ":" +
                        std::to_string(port) + "\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
      boost::asio::write(socket, boost::asio::buffer(req), ec);
      if (ec) return {false, ""};

      // Read full response
      boost::asio::streambuf resp_buf;
      boost::asio::read(socket, resp_buf, boost::asio::transfer_all(), ec);
      // ec == eof is expected

      std::istream stream(&resp_buf);
      std::string http_ver;
      int status_code = 0;
      stream >> http_ver >> status_code;
      std::string status_line_rest;
      std::getline(stream, status_line_rest);

      // Skip headers
      std::string line;
      while (std::getline(stream, line) && line != "\r" && !line.empty()) {
      }

      std::ostringstream body_stream;
      body_stream << stream.rdbuf();
      return {status_code >= 200 && status_code < 300, body_stream.str()};
    } catch (const std::exception &) {
      return {false, ""};
    }
  }

  void handleActivateMoveMode(std::shared_ptr<WebSocketConnection> connection)
  {
    std_msgs::msg::String injection_msg;
    injection_msg.data = "HALT";
    session_injection_pub_->publish(injection_msg);
    RCLCPP_INFO(this->get_logger(), "Move mode activated: injecting HALT session");
    json result = {{"type", "move_mode_activated"}, {"success", true}};
    connection->sendMessage(result.dump());
  }

  void handleMoveRobot(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    int robot_id = request.value("robot_id", -1);
    if (robot_id < 0 || robot_id > 15) {
      json error = {{"type", "error"}, {"message", "Invalid robot_id"}};
      connection->sendMessage(error.dump());
      return;
    }

    crane_msgs::msg::RobotCommands commands_msg;
    commands_msg.header.stamp = this->get_clock()->now();
    {
      std::lock_guard<std::mutex> lock(world_model_cache_mutex_);
      commands_msg.is_yellow = cached_is_yellow_;
      commands_msg.on_positive_half = cached_on_positive_half_;
    }

    crane_msgs::msg::RobotCommand cmd;
    cmd.robot_id = static_cast<uint8_t>(robot_id);
    cmd.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    cmd.target_theta = request.value("target_theta", 0.0f);

    crane_msgs::msg::PositionTargetMode pos_target;
    pos_target.target_x = request.value("target_x", 0.0f);
    pos_target.target_y = request.value("target_y", 0.0f);
    pos_target.position_tolerance = 0.05f;
    pos_target.speed_limit_at_target = 0.0f;
    cmd.position_target_mode.push_back(pos_target);

    commands_msg.robot_commands.push_back(cmd);
    move_command_pub_->publish(commands_msg);

    RCLCPP_INFO(
      this->get_logger(), "Move robot %d to (%.2f, %.2f)", robot_id, pos_target.target_x,
      pos_target.target_y);

    json result = {
      {"type", "move_robot_result"},
      {"robot_id", robot_id},
      {"success", true},
      {"target_x", pos_target.target_x},
      {"target_y", pos_target.target_y}};
    connection->sendMessage(result.dump());
  }

  void handleRobotControl(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    int robot_id = request.value("robot_id", -1);
    std::string command = request.value("command", "");

    if (robot_id < 0 || robot_id > 15 || command.empty()) {
      json error = {
        {"type", "robot_control_result"},
        {"robot_id", robot_id},
        {"command", command},
        {"success", false},
        {"status", "Invalid request"}};
      connection->sendMessage(error.dump());
      return;
    }

    static const std::map<std::string, std::pair<std::string, std::string>> kCommandMap = {
      {"start", {"POST", "/start"}}, {"stop", {"POST", "/stop"}}, {"status", {"GET", "/status"}}};

    auto it = kCommandMap.find(command);
    if (it == kCommandMap.end()) {
      json error = {
        {"type", "robot_control_result"},
        {"robot_id", robot_id},
        {"command", command},
        {"success", false},
        {"status", "Unknown command"}};
      connection->sendMessage(error.dump());
      return;
    }
    const auto & [method, path] = it->second;

    std::thread([connection, robot_id, command, method, path]() {
      constexpr int kPiPort = 8000;
      auto [success, body] = sendHttpRequest(getRobotIp(robot_id), kPiPort, method, path);
      json result = {
        {"type", "robot_control_result"},
        {"robot_id", robot_id},
        {"command", command},
        {"success", success},
        {"status", parseStatusFromBody(success, body)}};
      connection->sendMessage(result.dump());
    }).detach();
  }

  void pollAllPiStatus()
  {
    std::thread([this]() {
      constexpr int kNumRobots = 13;
      constexpr int kPiPort = 8000;

      std::vector<std::future<std::pair<int, std::string>>> futures;
      futures.reserve(kNumRobots);

      for (int id = 0; id < kNumRobots; id++) {
        futures.push_back(std::async(std::launch::async, [id]() {
          auto [success, body] = sendHttpRequest(getRobotIp(id), kPiPort, "GET", "/status");
          return std::make_pair(id, parseStatusFromBody(success, body, "Running", "Offline"));
        }));
      }

      json pi_status = {{"type", "pi_status"}, {"robots", json::array()}};
      for (auto & future : futures) {
        auto [id, status] = future.get();
        pi_status["robots"].push_back({{"robot_id", id}, {"status", status}});
      }
      broadcastToAll(pi_status.dump());
    }).detach();
  }

  void broadcastToAll(const std::string & message)
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto connection : connections_) {
      if (connection->isConnected()) {
        connection->sendMessage(message);
      }
    }
  }

  // ROS components
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub_;
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr robot_commands_sub_;
  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_sub_;
  rclcpp::Subscription<crane_visualization_interfaces::msg::SvgSnapshot>::SharedPtr
    aggregated_svgs_sub_;
  rclcpp::Subscription<crane_visualization_interfaces::msg::SvgUpdates>::SharedPtr
    visualizer_svgs_sub_;
  rclcpp::Publisher<crane_msgs::msg::HumanAnnotation>::SharedPtr annotation_pub_;
  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr move_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr session_injection_pub_;

  // Cached world model state for move commands
  bool cached_is_yellow_{false};
  bool cached_on_positive_half_{false};
  std::mutex world_model_cache_mutex_;

  // Robot monitoring
  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr robot_feedback_sub_;
  rclcpp::Subscription<crane_msgs::msg::PingStatusArray>::SharedPtr ping_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::TimerBase::SharedPtr robot_feedback_timer_;
  rclcpp::TimerBase::SharedPtr pi_status_timer_;
  crane_msgs::msg::RobotFeedbackArray::SharedPtr latest_robot_feedback_;
  bool robot_feedback_updated_{false};
  std::mutex robot_feedback_mutex_;

  // Control targets monitoring
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr control_targets_sub_;
  rclcpp::TimerBase::SharedPtr control_targets_timer_;
  crane_msgs::msg::RobotCommands::SharedPtr latest_control_targets_;
  bool control_targets_updated_{false};
  std::mutex control_targets_mutex_;

  // Server components
  std::thread http_thread_;
  std::thread websocket_thread_;
  std::string package_share_dir_;
  std::string web_root_;
  int port_;
  int websocket_port_;
  std::atomic<bool> running_{true};

  // WebSocket connections
  std::set<std::shared_ptr<WebSocketConnection>> connections_;
  std::mutex connections_mutex_;

  // Game info cache
  crane_msgs::msg::PlaySituation::SharedPtr latest_play_situation_;
  std::mutex game_info_mutex_;

  // World model throttle (10Hz)
  crane_msgs::msg::WorldModel::SharedPtr latest_world_model_;
  bool world_model_updated_{false};
  std::mutex world_model_throttle_mutex_;
  rclcpp::TimerBase::SharedPtr world_model_timer_;

  // SVG snapshot throttle (5Hz)
  crane_visualization_interfaces::msg::SvgSnapshot::SharedPtr latest_svg_snapshot_;
  bool svg_snapshot_updated_{false};
  std::mutex svg_snapshot_mutex_;
  rclcpp::TimerBase::SharedPtr svg_snapshot_timer_;

  // SVG updates coalescing (20Hz)
  std::vector<crane_visualization_interfaces::msg::SvgUpdates::SharedPtr> pending_svg_updates_;
  std::mutex svg_updates_mutex_;
  rclcpp::TimerBase::SharedPtr svg_updates_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WebSocketDebugServer>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
