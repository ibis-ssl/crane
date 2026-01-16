// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <openssl/evp.h>
#include <openssl/sha.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/asio.hpp>
#include <cctype>
#include <chrono>
#include <crane_msgs/msg/human_annotation.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/velocity_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_visualization_interfaces/msg/svg_snapshot.hpp>
#include <crane_visualization_interfaces/msg/svg_updates.hpp>
#include <filesystem>
#include <fstream>
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
      for (char c : message) {
        frame.push_back(static_cast<uint8_t>(c));
      }

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
      package_share_dir_ = ament_index_cpp::get_package_share_directory("crane_debug_tools");
      web_root_ = package_share_dir_ + "/web";
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Could not find package share directory: %s", e.what());
      web_root_ = "./web";
    }

    // Initialize subscribers
    world_model_sub_ = this->create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10,
      [this](const crane_msgs::msg::WorldModel::SharedPtr msg) { broadcastWorldModel(msg); });

    robot_commands_sub_ = this->create_subscription<crane_msgs::msg::VelocityCommands>(
      "/robot_commands", 10, [this](const crane_msgs::msg::VelocityCommands::SharedPtr msg) {
        broadcastRobotCommands(msg);
      });

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
          broadcastSvgData(msg);
        });

    // High-frequency incremental SVG updates
    visualizer_svgs_sub_ =
      this->create_subscription<crane_visualization_interfaces::msg::SvgUpdates>(
        "/visualizer_svgs", rclcpp::SensorDataQoS(),
        [this](const crane_visualization_interfaces::msg::SvgUpdates::SharedPtr msg) {
          broadcastSvgUpdates(msg);
        });

    // Human annotation publisher
    annotation_pub_ =
      this->create_publisher<crane_msgs::msg::HumanAnnotation>("/human_annotations", 10);

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
      } else if (type == "get_world_model") {
        // World model is automatically broadcasted, but we can send current state if needed
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

  void broadcastWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    json world_model = {
      {"type", "world_model"},
      {"timestamp", msg->header.stamp.sec * 1000000000L + msg->header.stamp.nanosec},
      {"is_yellow", msg->is_yellow},
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
    json delay_checkpoints_json = json::array();
    for (const auto & checkpoint : msg->delay_checkpoints.checkpoints) {
      delay_checkpoints_json.push_back(
        {{"name", checkpoint.name},
         {"relative_time_us", checkpoint.relative_time_us},
         {"value", checkpoint.value}});
    }
    world_model["delay_checkpoints"] = delay_checkpoints_json;
    world_model["delay_reference_timestamp_ns"] = msg->delay_checkpoints.reference_timestamp_ns;

    // WorldModel遅延分析情報
    if (!msg->delay_checkpoints.checkpoints.empty()) {
      json delay_analysis = {{"total_delay_ms", 0.0}, {"stage_delays", json::array()}};

      for (size_t i = 1; i < msg->delay_checkpoints.checkpoints.size(); ++i) {
        auto delay_us = msg->delay_checkpoints.checkpoints[i].relative_time_us -
                        msg->delay_checkpoints.checkpoints[i - 1].relative_time_us;
        auto delay_ms = static_cast<double>(delay_us) / 1000.0;

        delay_analysis["stage_delays"].push_back(
          {{"from", msg->delay_checkpoints.checkpoints[i - 1].name},
           {"to", msg->delay_checkpoints.checkpoints[i].name},
           {"delay_ms", delay_ms}});
      }

      if (msg->delay_checkpoints.checkpoints.size() > 1) {
        auto total_delay_us = msg->delay_checkpoints.checkpoints.back().relative_time_us;
        delay_analysis["total_delay_ms"] = static_cast<double>(total_delay_us) / 1000.0;
      }

      world_model["delay_analysis"] = delay_analysis;
    }

    broadcastToAll(world_model.dump());
  }

  void broadcastRobotCommands(const crane_msgs::msg::VelocityCommands::SharedPtr msg)
  {
    json commands = {{"type", "robot_commands"}, {"commands", json::array()}};

    // 遅延監視情報をRobotCommands全体レベルで追加
    json delay_checkpoints_json = json::array();
    for (const auto & checkpoint : msg->delay_checkpoints.checkpoints) {
      delay_checkpoints_json.push_back(
        {{"name", checkpoint.name},
         {"relative_time_us", checkpoint.relative_time_us},
         {"value", checkpoint.value}});
    }
    commands["delay_checkpoints"] = delay_checkpoints_json;
    commands["delay_reference_timestamp_ns"] = msg->delay_checkpoints.reference_timestamp_ns;

    // 遅延分析情報を計算して追加
    if (!msg->delay_checkpoints.checkpoints.empty()) {
      json delay_analysis = {{"total_delay_ms", 0.0}, {"stage_delays", json::array()}};

      for (size_t i = 1; i < msg->delay_checkpoints.checkpoints.size(); ++i) {
        auto delay_us = msg->delay_checkpoints.checkpoints[i].relative_time_us -
                        msg->delay_checkpoints.checkpoints[i - 1].relative_time_us;
        auto delay_ms = static_cast<double>(delay_us) / 1000.0;

        delay_analysis["stage_delays"].push_back(
          {{"from", msg->delay_checkpoints.checkpoints[i - 1].name},
           {"to", msg->delay_checkpoints.checkpoints[i].name},
           {"delay_ms", delay_ms}});
      }

      if (msg->delay_checkpoints.checkpoints.size() > 1) {
        auto total_delay_us = msg->delay_checkpoints.checkpoints.back().relative_time_us;
        delay_analysis["total_delay_ms"] = static_cast<double>(total_delay_us) / 1000.0;
      }

      commands["delay_analysis"] = delay_analysis;
    }

    for (const auto & cmd : msg->robot_commands) {
      json state_factors_json = json::array();
      for (const auto & factor : cmd.state_factors) {
        state_factors_json.push_back({{"name", factor.name}, {"state", factor.value}});
      }

      // 個別ロボットコマンドの遅延チェックポイント
      json cmd_delay_checkpoints_json = json::array();
      for (const auto & checkpoint : cmd.delay_checkpoints.checkpoints) {
        cmd_delay_checkpoints_json.push_back(
          {{"name", checkpoint.name},
           {"relative_time_us", checkpoint.relative_time_us},
           {"value", checkpoint.value}});
      }

      json cmd_json = {
        {"robot_id", cmd.robot_id},           {"kick_power", cmd.kick_power},
        {"dribble_power", cmd.dribble_power}, {"chip_enable", cmd.chip_enable},
        {"target_theta", cmd.target_theta},   {"state_factors", state_factors_json},
        {"planner_name", cmd.planner_name},   {"delay_checkpoints", cmd_delay_checkpoints_json}};
      commands["commands"].push_back(cmd_json);
    }

    broadcastToAll(commands.dump());
  }

  void broadcastSvgData(const crane_visualization_interfaces::msg::SvgSnapshot::SharedPtr msg)
  {
    const auto stamp_ns =
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
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

  void broadcastSvgUpdates(const crane_visualization_interfaces::msg::SvgUpdates::SharedPtr msg)
  {
    const auto stamp_ns =
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
    json svg_update = {
      {"type", "svg_update"},
      {"epoch", msg->epoch},
      {"seq", msg->seq},
      {"stamp_ns", stamp_ns},
      {"updates", json::array()}};

    for (const auto & upd : msg->updates) {
      json upd_json = {
        {"layer", upd.layer}, {"operation", upd.operation}, {"svg_primitives", json::array()}};
      for (const auto & primitive : upd.svg_primitives) {
        upd_json["svg_primitives"].push_back(primitive);
      }
      svg_update["updates"].push_back(upd_json);
    }

    broadcastToAll(svg_update.dump());
  }

  void handleTimeSyncRequest(
    std::shared_ptr<WebSocketConnection> connection, const json & request)
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
    // プレイ状況の文字列を取得
    std::string play_situation_str = msg->command.name;

    // スコアを取得
    int our_score = msg->our_team_info.score;
    int their_score = msg->their_team_info.score;

    // 試合時間を取得（秒単位）
    int game_time = msg->referee_raw.stage_time_left / 1000000;  // マイクロ秒から秒へ

    // ゲームステージを取得
    std::string game_stage = msg->stage.name;

    // ゲームイベント（ファールなど）を取得
    std::string game_event = msg->reason_text;

    json game_info = {
      {"type", "game_info"},
      {"play_situation", play_situation_str},
      {"our_score", our_score},
      {"their_score", their_score},
      {"game_time", game_time},
      {"game_stage", game_stage},
      {"game_event", game_event}};

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
  rclcpp::Subscription<crane_msgs::msg::VelocityCommands>::SharedPtr robot_commands_sub_;
  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_sub_;
  rclcpp::Subscription<crane_visualization_interfaces::msg::SvgSnapshot>::SharedPtr
    aggregated_svgs_sub_;
  rclcpp::Subscription<crane_visualization_interfaces::msg::SvgUpdates>::SharedPtr
    visualizer_svgs_sub_;
  rclcpp::Publisher<crane_msgs::msg::HumanAnnotation>::SharedPtr annotation_pub_;

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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WebSocketDebugServer>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
