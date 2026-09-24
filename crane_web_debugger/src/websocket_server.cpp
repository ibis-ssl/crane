// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <openssl/evp.h>
#include <openssl/sha.h>
#include <robocup_ssl_msgs/ssl_gc_common.pb.h>
#include <robocup_ssl_msgs/ssl_simulation_control.pb.h>
#include <sys/socket.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <crane_msgs/msg/human_annotation.hpp>
#include <crane_msgs/msg/latency_estimation_array.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/position_target_mode.hpp>
#include <crane_msgs/msg/robot_command.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_utils/package.hpp>
#include <crane_utils/parameter.hpp>
#include <crane_visualization_interfaces/msg/svg_snapshot.hpp>
#include <crane_visualization_interfaces/msg/svg_updates.hpp>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <future>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <thread>

using json = nlohmann::json;

// ---- SSL Simulation Protocol UDP sender ----
class SslSimulationSender
{
public:
  SslSimulationSender()
  : socket_(io_ctx_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
  {
    setEndpoint("127.0.0.1", 10300);
  }

  void setEndpoint(const std::string & host, int port)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    boost::asio::ip::udp::resolver resolver(io_ctx_);
    auto endpoints = resolver.resolve(host, std::to_string(port));
    endpoint_ = *endpoints.begin();
  }

  void sendTeleportBall(float x, float y, float vx, float vy)
  {
    robocup_ssl::SimulatorCommand cmd;
    auto * ball = cmd.mutable_control()->mutable_teleport_ball();
    ball->set_x(x);
    ball->set_y(y);
    ball->set_vx(vx);
    ball->set_vy(vy);
    sendCmd(cmd);
  }

  void sendTeleportRobot(int id, bool yellow, float x, float y, float orientation_rad, bool present)
  {
    robocup_ssl::SimulatorCommand cmd;
    auto * robot = cmd.mutable_control()->add_teleport_robot();
    robot->mutable_id()->set_id(static_cast<uint32_t>(id));
    robot->mutable_id()->set_team(yellow ? robocup_ssl::YELLOW : robocup_ssl::BLUE);
    robot->set_x(x);
    robot->set_y(y);
    robot->set_orientation(orientation_rad);
    robot->set_present(present);
    sendCmd(cmd);
  }

private:
  void sendCmd(const robocup_ssl::SimulatorCommand & cmd)
  {
    std::string data;
    if (!cmd.SerializeToString(&data)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      socket_.send_to(boost::asio::buffer(data), endpoint_);
    } catch (...) {
    }
  }

  boost::asio::io_context io_ctx_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;
  std::mutex mutex_;
};

class WebSocketConnection
{
public:
  explicit WebSocketConnection(std::shared_ptr<boost::asio::ip::tcp::socket> socket)
  : socket_(socket), connected_(false)
  {
  }

  // コピー/ムーブ禁止（mutexはコピー不可）
  WebSocketConnection(const WebSocketConnection &) = delete;
  WebSocketConnection & operator=(const WebSocketConnection &) = delete;

  bool handshake()
  {
    try {
      boost::asio::streambuf buffer;
      boost::asio::read_until(*socket_, buffer, "\r\n\r\n");

      std::istream request_stream(&buffer);
      std::string line;
      std::string websocket_key;

      while (std::getline(request_stream, line) && line != "\r") {
        if (line.starts_with("Sec-WebSocket-Key:")) {
          websocket_key = line.substr(19);
          websocket_key.erase(0, websocket_key.find_first_not_of(" \t\r\n"));
          websocket_key.erase(websocket_key.find_last_not_of(" \t\r\n") + 1);
        }
      }

      if (websocket_key.empty()) {
        return false;
      }

      std::string accept_key = generateAcceptKey(websocket_key);

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

  void sendMessage(const std::string & message) { sendFrame(kOpText, message); }

  // 次のテキスト/バイナリメッセージを返す。ping には pong を返し、pong は読み捨てる。
  // 断片化されたメッセージは FIN まで連結する。close 受信・切断・プロトコル違反なら nullopt
  std::optional<std::string> receiveMessage()
  {
    std::string message;
    bool in_fragmented_message = false;
    try {
      while (connected_) {
        uint8_t header[2];
        boost::asio::read(*socket_, boost::asio::buffer(header, 2));
        const bool fin = (header[0] & 0x80) != 0;
        const uint8_t opcode = header[0] & 0x0F;
        const bool masked = (header[1] & 0x80) != 0;
        uint64_t payload_length = header[1] & 0x7F;

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

        // 制御フレームは分割できず 125 バイト以下（RFC 6455 5.5）
        const bool is_control = (opcode & 0x08) != 0;
        if (is_control && (!fin || payload_length > 125)) {
          closeWithStatus(kCloseProtocolError);
          return std::nullopt;
        }
        // 長さはクライアントが決める値なので、確保する前に上限で切る
        if (!is_control && message.size() + payload_length > kMaxMessageBytes) {
          closeWithStatus(kCloseMessageTooBig);
          return std::nullopt;
        }

        uint8_t mask[4] = {0, 0, 0, 0};
        if (masked) {
          boost::asio::read(*socket_, boost::asio::buffer(mask, 4));
        }
        std::string payload(payload_length, '\0');
        if (payload_length > 0) {
          boost::asio::read(*socket_, boost::asio::buffer(payload));
          if (masked) {
            for (uint64_t i = 0; i < payload_length; i++) {
              payload[i] = static_cast<char>(payload[i] ^ mask[i % 4]);
            }
          }
        }

        switch (opcode) {
          case kOpClose:
            // 受け取ったステータスコードをそのまま返して閉じる
            sendFrame(kOpClose, payload.substr(0, 2));
            connected_ = false;
            return std::nullopt;
          case kOpPing:
            sendFrame(kOpPong, payload);
            continue;
          case kOpPong:
            continue;
          case kOpText:
          case kOpBinary:
            if (in_fragmented_message) {
              closeWithStatus(kCloseProtocolError);
              return std::nullopt;
            }
            message = std::move(payload);
            break;
          case kOpContinuation:
            if (!in_fragmented_message) {
              closeWithStatus(kCloseProtocolError);
              return std::nullopt;
            }
            message += payload;
            break;
          default:
            closeWithStatus(kCloseProtocolError);
            return std::nullopt;
        }
        if (fin) return message;
        in_fragmented_message = true;
      }
    } catch (const std::exception &) {
      connected_ = false;
    }
    return std::nullopt;
  }

  bool isConnected() const { return connected_; }

private:
  static constexpr uint8_t kOpContinuation = 0x0;
  static constexpr uint8_t kOpText = 0x1;
  static constexpr uint8_t kOpBinary = 0x2;
  static constexpr uint8_t kOpClose = 0x8;
  static constexpr uint8_t kOpPing = 0x9;
  static constexpr uint8_t kOpPong = 0xA;
  static constexpr uint16_t kCloseProtocolError = 1002;
  static constexpr uint16_t kCloseMessageTooBig = 1009;
  // クライアントから来るのは注釈や操作要求の小さな JSON だけ
  static constexpr uint64_t kMaxMessageBytes = 1 << 20;

  void sendFrame(uint8_t opcode, const std::string & payload)
  {
    std::lock_guard<std::mutex> lock(send_mutex_);

    if (!connected_) return;

    try {
      std::vector<uint8_t> frame;
      frame.push_back(0x80 | opcode);  // FIN=1

      if (payload.length() < 126) {
        frame.push_back(static_cast<uint8_t>(payload.length()));
      } else if (payload.length() < 65536) {
        frame.push_back(126);
        frame.push_back((payload.length() >> 8) & 0xFF);
        frame.push_back(payload.length() & 0xFF);
      } else {
        frame.push_back(127);
        for (int i = 7; i >= 0; i--) {
          frame.push_back((payload.length() >> (i * 8)) & 0xFF);
        }
      }

      frame.insert(frame.end(), payload.begin(), payload.end());

      boost::asio::write(*socket_, boost::asio::buffer(frame));
    } catch (const std::exception & e) {
      connected_ = false;
    }
  }

  void closeWithStatus(uint16_t status)
  {
    const std::string payload{static_cast<char>(status >> 8), static_cast<char>(status & 0xFF)};
    sendFrame(kOpClose, payload);
    connected_ = false;
  }

  std::string generateAcceptKey(const std::string & key)
  {
    // WebSocket GUID as per RFC 6455
    const std::string websocket_magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    std::string concat = key + websocket_magic;

    unsigned char hash[SHA_DIGEST_LENGTH];
    SHA1(reinterpret_cast<const unsigned char *>(concat.c_str()), concat.length(), hash);

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
  std::atomic<bool> connected_;
  std::mutex send_mutex_;
};

// 最新のメッセージだけを残し、タイマーで間引いて配信するためのバッファ
template <typename MsgT>
class LatestMessage
{
public:
  using Ptr = typename MsgT::SharedPtr;

  // 保存し、これが初めての受信だったかを返す
  bool store(const Ptr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool first = (latest_ == nullptr);
    latest_ = msg;
    updated_ = true;
    return first;
  }

  // 前回取り出してから更新があれば最新を返し、なければ nullptr を返す
  Ptr takeIfUpdated()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!updated_ || !latest_) return nullptr;
    updated_ = false;
    return latest_;
  }

  Ptr latest() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
  }

private:
  mutable std::mutex mutex_;
  Ptr latest_;
  bool updated_{false};
};

class WebSocketDebugServer : public rclcpp::Node
{
public:
  WebSocketDebugServer() : Node("websocket_debug_server")
  {
    websocket_port_ = crane::get_or_declare_parameter(this, "websocket_port", 8091);

    world_model_sub_ = this->create_subscription<crane_msgs::msg::WorldModel>(
      "/world_model", 10, [this](const crane_msgs::msg::WorldModel::SharedPtr msg) {
        const bool first_msg = world_model_.store(msg);
        // 初回受信時は全クライアントへ即座にブロードキャスト（10Hzタイマー待ち不要）
        if (first_msg) {
          RCLCPP_INFO(this->get_logger(), "world_model 初回受信 - 即座にブロードキャスト");
          broadcastWorldModel(msg);
        }
      });

    robot_commands_sub_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/robot_commands", 10,
      [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) { broadcastRobotCommands(msg); });

    play_situation_sub_ = this->create_subscription<crane_msgs::msg::PlaySituation>(
      "/play_situation", 10, [this](const crane_msgs::msg::PlaySituation::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(game_info_mutex_);
          latest_play_situation_ = msg;
        }
        broadcastToAll(createGameInfoMessage(msg));
      });

    aggregated_svgs_sub_ =
      this->create_subscription<crane_visualization_interfaces::msg::SvgSnapshot>(
        "/aggregated_svgs", 10,
        [this](const crane_visualization_interfaces::msg::SvgSnapshot::SharedPtr msg) {
          svg_snapshot_.store(msg);
        });

    // High-frequency incremental SVG updates — accumulate, flush at 20Hz
    visualizer_svgs_sub_ =
      this->create_subscription<crane_visualization_interfaces::msg::SvgUpdates>(
        "/visualizer_svgs", rclcpp::SensorDataQoS(),
        [this](const crane_visualization_interfaces::msg::SvgUpdates::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(svg_updates_mutex_);
          pending_svg_updates_.push_back(msg);
        });

    annotation_pub_ =
      this->create_publisher<crane_msgs::msg::HumanAnnotation>("/human_annotations", 10);

    move_command_pub_ =
      this->create_publisher<crane_msgs::msg::RobotCommands>("/control_targets", 10);
    session_injection_pub_ =
      this->create_publisher<std_msgs::msg::String>("/session_injection", 10);
    session_injection_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/session_injection", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(injection_mutex_);
          current_injection_ = msg->data;
          auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
          injection_history_.push_front({msg->data, ms});
          while (injection_history_.size() > 5) injection_history_.pop_back();
        }
        broadcastSessionInjectionCurrent();
      });
    loadSituationNames();
    robot_test_target_pub_ =
      this->create_publisher<crane_msgs::msg::RobotCommand>("/robot_test/target", 10);
    local_planner_params_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(this, "local_planner");
    ibis_sender_params_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(this, "ibis_sender");

    // Robot feedback subscription (cached, broadcast at 10Hz via timer)
    robot_feedback_sub_ = this->create_subscription<crane_msgs::msg::RobotFeedbackArray>(
      "/robot_feedback", 10, [this](const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg) {
        robot_feedback_.store(msg);
      });

    ping_sub_ = this->create_subscription<crane_msgs::msg::PingStatusArray>(
      "/ping", 10,
      [this](const crane_msgs::msg::PingStatusArray::SharedPtr msg) { broadcastPingStatus(msg); });

    diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics_agg", 10, [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        broadcastDiagnostics(msg);
      });

    // Latency estimation subscription (cached, broadcast at 10Hz via timer)
    latency_estimation_sub_ = this->create_subscription<crane_msgs::msg::LatencyEstimationArray>(
      "/latency_estimation", 10,
      [this](const crane_msgs::msg::LatencyEstimationArray::SharedPtr msg) {
        latency_estimation_.store(msg);
      });

    latency_estimation_timer_ = createThrottleTimer(
      std::chrono::milliseconds(100), latency_estimation_,
      [this](const auto & msg) { broadcastLatencyEstimation(msg); });

    robot_feedback_timer_ = createThrottleTimer(
      std::chrono::milliseconds(100), robot_feedback_,
      [this](const auto & msg) { broadcastRobotFeedback(msg); });

    // control_targets subscription (cached, broadcast at 10Hz via timer)
    control_targets_sub_ = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/control_targets", 10,
      [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) { control_targets_.store(msg); });

    control_targets_timer_ = createThrottleTimer(
      std::chrono::milliseconds(100), control_targets_,
      [this](const auto & msg) { broadcastControlTargets(msg); });

    world_model_timer_ = createThrottleTimer(
      std::chrono::milliseconds(100), world_model_,
      [this](const auto & msg) { broadcastWorldModel(msg); });

    svg_snapshot_timer_ = createThrottleTimer(
      std::chrono::milliseconds(200), svg_snapshot_,
      [this](const auto & msg) { broadcastSvgData(msg); });

    svg_updates_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
      std::vector<crane_visualization_interfaces::msg::SvgUpdates::SharedPtr> batch;
      {
        std::lock_guard<std::mutex> lock(svg_updates_mutex_);
        if (pending_svg_updates_.empty()) return;
        batch.swap(pending_svg_updates_);
      }
      broadcastCoalescedSvgUpdates(batch);
    });

    try {
      ws_acceptor_ = std::make_unique<boost::asio::ip::tcp::acceptor>(
        ws_io_context_,
        boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), websocket_port_));
      RCLCPP_INFO(this->get_logger(), "WebSocket server listening on port %d", websocket_port_);
      websocket_thread_ = std::thread([this]() { this->runWebSocketServer(); });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "WebSocket server error: %s", e.what());
    }

    RCLCPP_INFO(this->get_logger(), "WebSocket: ws://localhost:%d", websocket_port_);
  }

  ~WebSocketDebugServer() { stopServers(); }

private:
  void stopServers()
  {
    running_ = false;

    if (ws_acceptor_) {
      // close() だけではブロック中の accept() が戻らない。shutdown で起こす
      ::shutdown(ws_acceptor_->native_handle(), SHUT_RDWR);
      boost::system::error_code ec;
      ws_acceptor_->close(ec);
    }
    ws_io_context_.stop();

    if (websocket_thread_.joinable()) {
      websocket_thread_.join();
    }

    // 接続スレッドは this を握っている。ノードを壊す前に全員を受信待ちから起こして抜けさせる
    std::unique_lock<std::mutex> lock(sockets_mutex_);
    for (const auto & socket : sockets_) {
      boost::system::error_code ec;
      socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    }
    if (!sockets_cv_.wait_for(lock, std::chrono::seconds(3), [this] { return sockets_.empty(); })) {
      RCLCPP_WARN(
        this->get_logger(), "%zu WebSocket connection thread(s) did not exit", sockets_.size());
    }
  }

  void runWebSocketServer()
  {
    try {
      while (running_) {
        auto socket = std::make_shared<boost::asio::ip::tcp::socket>(ws_io_context_);
        boost::system::error_code ec;
        ws_acceptor_->accept(*socket, ec);
        if (ec) {
          if (!running_ || ec == boost::asio::error::operation_aborted) {
            break;
          }
          RCLCPP_WARN(this->get_logger(), "WebSocket accept failed: %s", ec.message().c_str());
          continue;
        }

        {
          std::lock_guard<std::mutex> lock(sockets_mutex_);
          sockets_.insert(socket);
        }
        std::thread([this, socket]() {
          handleWebSocketConnection(socket);
          std::lock_guard<std::mutex> lock(sockets_mutex_);
          sockets_.erase(socket);
          sockets_cv_.notify_all();
        }).detach();
      }
    } catch (const std::exception & e) {
      if (running_) {
        RCLCPP_ERROR(this->get_logger(), "WebSocket server error: %s", e.what());
      }
    }
  }

  template <typename MsgT, typename Broadcast>
  rclcpp::TimerBase::SharedPtr createThrottleTimer(
    std::chrono::milliseconds period, LatestMessage<MsgT> & buffer, Broadcast broadcast)
  {
    return this->create_wall_timer(period, [&buffer, broadcast]() {
      if (auto msg = buffer.takeIfUpdated()) broadcast(msg);
    });
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

    {
      crane_msgs::msg::PlaySituation::SharedPtr play_situation;
      {
        std::lock_guard<std::mutex> lock(game_info_mutex_);
        play_situation = latest_play_situation_;
      }
      if (play_situation) {
        connection->sendMessage(createGameInfoMessage(play_situation));
      }
    }

    {
      if (auto wm_msg = world_model_.latest()) {
        connection->sendMessage(createWorldModelMessage(wm_msg));
      }
    }

    {
      if (auto fb_msg = robot_feedback_.latest()) {
        connection->sendMessage(createRobotFeedbackMessage(fb_msg));
      }
    }

    handleListSituations(connection);
    connection->sendMessage(createSessionInjectionCurrentMessage());

    while (running_) {
      auto message = connection->receiveMessage();
      if (!message) break;
      if (!message->empty()) handleWebSocketMessage(connection, *message);
    }

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
      const std::string type = request.value("type", "");

      if (type == "time_sync_request") {
        handleTimeSyncRequest(connection, request);
      } else if (type == "annotation") {
        handleAnnotation(connection, request);
      } else if (type == "activate_move_mode") {
        handleActivateMoveMode(connection);
      } else if (type == "move_robot") {
        handleMoveRobot(connection, request);
      } else if (type == "activate_robot_test") {
        handleActivateRobotTest(connection, request);
      } else if (type == "deactivate_robot_test") {
        handleDeactivateRobotTest(connection);
      } else if (type == "robot_test_target") {
        handleRobotTestTarget(connection, request);
      } else if (type == "set_planner_param") {
        handleSetPlannerParam(connection, request);
      } else if (type == "get_position_control_config") {
        handleGetPositionControlConfig(connection);
      } else if (type == "set_position_control_param") {
        handleSetPositionControlParam(connection, request);
      } else if (type == "sim_teleport_ball") {
        handleSimTeleportBall(connection, request);
      } else if (type == "sim_teleport_robot") {
        handleSimTeleportRobot(connection, request);
      } else if (type == "sim_remove_robot") {
        handleSimRemoveRobot(connection, request);
      } else if (type == "sim_set_endpoint") {
        handleSimSetEndpoint(connection, request);
      } else if (type == "list_situations") {
        handleListSituations(connection);
      } else if (type == "session_inject") {
        handleSessionInject(connection, request);
      } else if (type == "session_clear") {
        std_msgs::msg::String injection_msg;
        injection_msg.data = "HALT";
        session_injection_pub_->publish(injection_msg);
        RCLCPP_INFO(this->get_logger(), "Session injection cleared (HALT)");
        json result = {{"type", "session_inject_result"}, {"success", true}, {"name", "HALT"}};
        connection->sendMessage(result.dump());
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

  static std::string createWorldModelMessage(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    json world_model = {
      {"type", "world_model"},
      {"timestamp", msg->header.stamp.sec * 1000000000L + msg->header.stamp.nanosec},
      {"is_yellow", msg->is_yellow},
      {"on_positive_half", msg->on_positive_half},
      {"field_info", {{"length", msg->field_info.x}, {"width", msg->field_info.y}}},
      {"penalty_area_size",
       {{"depth", msg->penalty_area_size.x}, {"width", msg->penalty_area_size.y}}},
      {"goal_size", {{"depth", msg->goal_size.x}, {"width", msg->goal_size.y}}},
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

    world_model["delay_checkpoints"] =
      to_delay_checkpoints_json(msg->delay_checkpoints.checkpoints);
    world_model["delay_reference_timestamp_ns"] = msg->delay_checkpoints.reference_timestamp_ns;

    if (!msg->delay_checkpoints.checkpoints.empty()) {
      world_model["delay_analysis"] =
        compute_delay_analysis_json(msg->delay_checkpoints.checkpoints);
    }

    return world_model.dump();
  }

  void broadcastWorldModel(const crane_msgs::msg::WorldModel::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(world_model_cache_mutex_);
      cached_is_yellow_ = msg->is_yellow;
      cached_on_positive_half_ = msg->on_positive_half;
    }

    broadcastToAll(createWorldModelMessage(msg));
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
        {"speed_limit_at_target", pt.speed_limit_at_target},
        {"terminal_velocity_x", pt.terminal_velocity_x},
        {"terminal_velocity_y", pt.terminal_velocity_y}};
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
      {"client_send_time_ms", request.at("client_send_time_ms")},
      {"server_receive_time_ms", T2},
      {"server_send_time_ms", T2},  // 処理時間が短いため同じ
      {"ros_time_ns", this->get_clock()->now().nanoseconds()}};

    connection->sendMessage(response.dump());
  }

  void handleAnnotation(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    auto msg = crane_msgs::msg::HumanAnnotation();

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "human_annotation";

    msg.category = request.value("category", 0);
    msg.priority = request.value("priority", 1);
    msg.label = request.value("label", "");
    msg.description = request.value("description", "");

    msg.event_timestamp_ns = request.value("event_timestamp_ns", 0L);
    msg.client_timestamp_ms = request.value("client_timestamp_ms", 0L);
    msg.time_offset_ms = request.value("time_offset_ms", 0);

    int64_t default_timestamp = this->get_clock()->now().nanoseconds();
    if (msg.event_timestamp_ns == 0) {
      msg.event_timestamp_ns = default_timestamp;
    }

    if (request.contains("position")) {
      msg.has_position = true;
      msg.position.x = request["position"].value("x", 0.0);
      msg.position.y = request["position"].value("y", 0.0);
      msg.position.z = request["position"].value("z", 0.0);
    } else {
      msg.has_position = false;
    }

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

    if (request.contains("metadata")) {
      msg.metadata_json = request["metadata"].dump();
    }

    annotation_pub_->publish(msg);

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

  static std::string createRobotFeedbackMessage(
    const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg)
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
    return data.dump();
  }

  void broadcastRobotFeedback(const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg)
  {
    broadcastToAll(createRobotFeedbackMessage(msg));
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

  void broadcastLatencyEstimation(const crane_msgs::msg::LatencyEstimationArray::SharedPtr msg)
  {
    json data = {{"type", "latency_estimation"}, {"estimations", json::array()}};
    for (const auto & est : msg->estimations) {
      const float lms = est.latency_ms;
      data["estimations"].push_back(
        {{"robot_id", est.robot_id},
         {"source", est.source},
         {"latency_ms", std::isnan(lms) ? json(nullptr) : json(lms)},
         {"correlation", est.correlation},
         {"samples_used", est.samples_used},
         {"cmd_stddev", est.cmd_stddev}});
    }
    broadcastToAll(data.dump());
  }

  void loadSituationNames()
  {
    try {
      auto config_path = crane::resolve_package_path(
        this->get_logger(), "crane_session_coordinator", "unified_session_config.yaml");
      YAML::Node config = YAML::LoadFile(config_path.string());
      std::lock_guard<std::mutex> lock(injection_mutex_);
      cached_situation_names_.clear();
      if (config["situations"]) {
        for (auto it = config["situations"].begin(); it != config["situations"].end(); ++it) {
          cached_situation_names_.push_back(it->first.as<std::string>());
        }
      }
      RCLCPP_INFO(
        this->get_logger(), "Loaded %zu situations from config", cached_situation_names_.size());
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to load situation names: %s", e.what());
    }
  }

  void handleListSituations(std::shared_ptr<WebSocketConnection> connection)
  {
    json items = json::array();
    {
      std::lock_guard<std::mutex> lock(injection_mutex_);
      for (const auto & name : cached_situation_names_) {
        items.push_back(name);
      }
    }
    json response = {{"type", "situations_list"}, {"items", items}};
    connection->sendMessage(response.dump());
  }

  void handleSessionInject(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    std::string name = request.value("name", "HALT");
    std_msgs::msg::String injection_msg;
    injection_msg.data = name;
    session_injection_pub_->publish(injection_msg);
    RCLCPP_INFO(this->get_logger(), "Session injected: %s", name.c_str());
    json result = {{"type", "session_inject_result"}, {"success", true}, {"name", name}};
    connection->sendMessage(result.dump());
  }

  std::string createSessionInjectionCurrentMessage()
  {
    json history_json = json::array();
    std::string current;
    {
      std::lock_guard<std::mutex> lock(injection_mutex_);
      current = current_injection_;
      for (const auto & [n, ts] : injection_history_) {
        history_json.push_back({{"name", n}, {"timestamp_ms", ts}});
      }
    }
    json msg = {
      {"type", "session_injection_current"}, {"name", current}, {"history", history_json}};
    return msg.dump();
  }

  void broadcastSessionInjectionCurrent()
  {
    broadcastToAll(createSessionInjectionCurrentMessage());
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

  // robot_test セッション向けの位置指令。max_velocity / max_acceleration は request から読む
  static crane_msgs::msg::RobotCommand makeRobotTestCommand(
    int robot_id, float x, float y, double theta, const json & request)
  {
    crane_msgs::msg::RobotCommand cmd;
    cmd.robot_id = static_cast<uint8_t>(robot_id);
    cmd.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
    cmd.target_theta = theta;

    crane_msgs::msg::PositionTargetMode pos_target;
    pos_target.target_x = x;
    pos_target.target_y = y;
    pos_target.position_tolerance = 0.05f;
    pos_target.speed_limit_at_target = 0.0f;
    cmd.position_target_mode.push_back(pos_target);

    crane_msgs::msg::NamedFloat vel_factor;
    vel_factor.name = "robot_test";
    vel_factor.value = static_cast<float>(request.value("max_velocity", 2.0));
    cmd.local_planner_config.max_velocity_factors.push_back(vel_factor);

    crane_msgs::msg::NamedFloat acc_factor;
    acc_factor.name = "robot_test";
    acc_factor.value = static_cast<float>(request.value("max_acceleration", 2.5));
    cmd.local_planner_config.max_acceleration_factors.push_back(acc_factor);
    return cmd;
  }

  void handleActivateRobotTest(
    std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    int robot_id = request.value("robot_id", -1);
    if (robot_id >= 0 && robot_id <= 15) {
      double x = 0.0, y = 0.0, theta = 0.0;
      if (const auto world_model = world_model_.latest()) {
        for (const auto & r : world_model->robot_info_ours) {
          if (r.id == robot_id) {
            x = r.pose.x;
            y = r.pose.y;
            theta = r.pose.theta;
            break;
          }
        }
      }
      robot_test_target_pub_->publish(makeRobotTestCommand(
        robot_id, static_cast<float>(x), static_cast<float>(y), theta, request));
      RCLCPP_INFO(
        this->get_logger(), "Robot test mode target initialized: robot=%d at (%.2f, %.2f)",
        robot_id, x, y);
    }

    std_msgs::msg::String injection_msg;
    injection_msg.data = "ROBOT_TEST";
    session_injection_pub_->publish(injection_msg);
    RCLCPP_INFO(this->get_logger(), "Robot test mode activated: injecting ROBOT_TEST session");
    json result = {{"type", "robot_test_activated"}, {"success", true}};
    connection->sendMessage(result.dump());
  }

  void handleDeactivateRobotTest(std::shared_ptr<WebSocketConnection> connection)
  {
    std_msgs::msg::String injection_msg;
    injection_msg.data = "HALT";
    session_injection_pub_->publish(injection_msg);
    RCLCPP_INFO(this->get_logger(), "Robot test mode deactivated: injecting HALT session");
    json result = {{"type", "robot_test_deactivated"}, {"success", true}};
    connection->sendMessage(result.dump());
  }

  void handleRobotTestTarget(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    int robot_id = request.value("robot_id", -1);
    if (robot_id < 0 || robot_id > 15) {
      json error = {{"type", "error"}, {"message", "Invalid robot_id"}};
      connection->sendMessage(error.dump());
      return;
    }

    const float target_x = request.value("target_x", 0.0f);
    const float target_y = request.value("target_y", 0.0f);
    robot_test_target_pub_->publish(makeRobotTestCommand(
      robot_id, target_x, target_y, request.value("target_theta", 0.0), request));

    RCLCPP_DEBUG(
      this->get_logger(), "Robot test target: robot=%d, x=%.2f, y=%.2f", robot_id, target_x,
      target_y);

    json result = {{"type", "robot_test_target_result"}, {"robot_id", robot_id}, {"success", true}};
    connection->sendMessage(result.dump());
  }

  void handleSetPlannerParam(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    std::vector<rclcpp::Parameter> params;
    if (request.contains("velocity_damping_gain")) {
      double v = std::clamp(request.value("velocity_damping_gain", 0.5), 0.0, 2.0);
      params.emplace_back("velocity_damping_gain", v);
    }
    if (params.empty()) {
      json err = {{"type", "error"}, {"message", "no known param"}};
      connection->sendMessage(err.dump());
      return;
    }
    if (!local_planner_params_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "local_planner parameter service not ready");
      json err = {
        {"type", "set_planner_param_result"}, {"success", false}, {"message", "not ready"}};
      connection->sendMessage(err.dump());
      return;
    }
    local_planner_params_client_->set_parameters(
      params,
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        try {
          for (const auto & r : future.get()) {
            if (!r.successful) {
              RCLCPP_WARN(this->get_logger(), "set param failed: %s", r.reason.c_str());
            }
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(this->get_logger(), "set param exception: %s", e.what());
        }
      });
    json ok = {{"type", "set_planner_param_result"}, {"success", true}};
    connection->sendMessage(ok.dump());
  }

  /**
   * 位置制御ゲインの遠隔調整（ibis-ssl/crane#1442）
   *
   * ibis_sender は 1 秒ごとに position_control.* を get_parameter() で読み直し、
   * 28 バイト（v2）の設定パケットとして CM4 へ送る（Orion_CM4
   * cm4/bridge/config_packet.h、UDP 12350）。位置制御ループ自体は CM4 側で
   * 閉じているので、ここでパラメータを書き換えることがロボットのゲインを
   * 稼働中に変える唯一の経路になる。ロボットの再起動は要らない。
   */
  struct PositionControlParamSpec
  {
    const char * name;
    double min_value;
    double max_value;
  };

  // 範囲は CM4 側の受理範囲（config_packet.h の kConfig*Max）に合わせる。
  // CM4 は範囲外の値をクランプせずデータグラムごと捨て、拒否理由は CM4 の
  // ログにしか出ない。crane 側で弾かないと「設定できたのに何も起きない」
  // という無言の失敗になるので、ここが最後の防波堤になる。
  static constexpr std::array<PositionControlParamSpec, 5> kPositionControlParams{
    {{"position_control.kp", 0.0, 20.0},
     {"position_control.ki", 0.0, 20.0},
     {"position_control.kd", 0.0, 5.0},
     {"position_control.deceleration", 0.0, 20.0},
     {"position_control.tolerance", 0.0, 1.0}}};

  void handleGetPositionControlConfig(std::shared_ptr<WebSocketConnection> connection)
  {
    if (!ibis_sender_params_client_->service_is_ready()) {
      json err = {
        {"type", "position_control_config"},
        {"ready", false},
        {"message", "ibis_sender のパラメータサービスが応答していません"}};
      connection->sendMessage(err.dump());
      return;
    }

    std::vector<std::string> names;
    names.reserve(kPositionControlParams.size());
    for (const auto & spec : kPositionControlParams) {
      names.emplace_back(spec.name);
    }

    ibis_sender_params_client_->get_parameters(
      names, [this, connection](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        json msg = {{"type", "position_control_config"}, {"ready", true}};
        try {
          json values = json::object();
          for (const auto & param : future.get()) {
            switch (param.get_type()) {
              case rclcpp::ParameterType::PARAMETER_DOUBLE:
                values[param.get_name()] = param.as_double();
                break;
              case rclcpp::ParameterType::PARAMETER_STRING:
                values[param.get_name()] = param.as_string();
                break;
              default:
                break;
            }
          }
          json limits = json::object();
          for (const auto & spec : kPositionControlParams) {
            limits[spec.name] = {{"min", spec.min_value}, {"max", spec.max_value}};
          }
          msg["values"] = values;
          msg["limits"] = limits;
        } catch (const std::exception & e) {
          msg["ready"] = false;
          msg["message"] = e.what();
          RCLCPP_WARN(this->get_logger(), "位置制御パラメータの読み出しに失敗: %s", e.what());
        }
        connection->sendMessage(msg.dump());
      });
  }

  void handleSetPositionControlParam(
    std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    const std::string name = request.value("name", "");
    const auto spec = std::find_if(
      kPositionControlParams.begin(), kPositionControlParams.end(),
      [&name](const PositionControlParamSpec & s) { return name == s.name; });

    auto reject = [&](const std::string & reason) {
      json err = {
        {"type", "set_position_control_param_result"},
        {"name", name},
        {"success", false},
        {"message", reason}};
      connection->sendMessage(err.dump());
    };

    if (spec == kPositionControlParams.end()) {
      reject("未知のパラメータです: " + name);
      return;
    }
    if (!request.contains("value") || !request["value"].is_number()) {
      reject("value が数値ではありません");
      return;
    }

    const double value = request["value"].get<double>();
    if (!std::isfinite(value) || value < spec->min_value || value > spec->max_value) {
      // クランプしない。丸めた値で「適用済み」と表示すると、UI の表示と
      // ロボットの実効値が食い違ったまま気付けなくなる（CM4 側と同じ方針）。
      std::ostringstream oss;
      oss << "範囲外の値です (" << spec->min_value << " 〜 " << spec->max_value << ")";
      reject(oss.str());
      return;
    }
    if (!ibis_sender_params_client_->service_is_ready()) {
      reject("ibis_sender のパラメータサービスが応答していません");
      return;
    }

    ibis_sender_params_client_->set_parameters(
      {rclcpp::Parameter(name, value)},
      [this, connection,
       name](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        json msg = {{"type", "set_position_control_param_result"}, {"name", name}};
        try {
          const auto results = future.get();
          const bool ok = !results.empty() && results.front().successful;
          msg["success"] = ok;
          if (!ok) {
            const std::string reason = results.empty() ? "結果が空です" : results.front().reason;
            msg["message"] = reason;
            RCLCPP_WARN(
              this->get_logger(), "位置制御パラメータの設定に失敗 (%s): %s", name.c_str(),
              reason.c_str());
          }
        } catch (const std::exception & e) {
          msg["success"] = false;
          msg["message"] = e.what();
          RCLCPP_WARN(this->get_logger(), "位置制御パラメータの設定で例外: %s", e.what());
        }
        connection->sendMessage(msg.dump());
      });
  }

  void handleSimTeleportBall(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    float x = static_cast<float>(request.value("x", 0.0));
    float y = static_cast<float>(request.value("y", 0.0));
    float vx = static_cast<float>(request.value("vx", 0.0));
    float vy = static_cast<float>(request.value("vy", 0.0));
    sim_sender_.sendTeleportBall(x, y, vx, vy);
    RCLCPP_DEBUG(this->get_logger(), "Sim teleport ball to (%.2f, %.2f)", x, y);
    json result = {{"type", "sim_result"}, {"success", true}};
    connection->sendMessage(result.dump());
  }

  void handleSimTeleportRobot(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    int id = request.value("id", 0);
    std::string team = request.value("team", "blue");
    bool yellow = (team == "yellow");
    float x = static_cast<float>(request.value("x", 0.0));
    float y = static_cast<float>(request.value("y", 0.0));
    float orientation_deg = static_cast<float>(request.value("orientation_deg", 0.0));
    float orientation_rad = orientation_deg * static_cast<float>(M_PI) / 180.0f;
    bool present = request.value("present", true);
    sim_sender_.sendTeleportRobot(id, yellow, x, y, orientation_rad, present);
    RCLCPP_DEBUG(
      this->get_logger(), "Sim teleport robot %s #%d to (%.2f, %.2f)", team.c_str(), id, x, y);
    json result = {{"type", "sim_result"}, {"success", true}};
    connection->sendMessage(result.dump());
  }

  void handleSimRemoveRobot(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    int id = request.value("id", 0);
    std::string team = request.value("team", "blue");
    bool yellow = (team == "yellow");
    sim_sender_.sendTeleportRobot(id, yellow, 0.0f, 0.0f, 0.0f, false);
    json result = {{"type", "sim_result"}, {"success", true}};
    connection->sendMessage(result.dump());
  }

  void handleSimSetEndpoint(std::shared_ptr<WebSocketConnection> connection, const json & request)
  {
    std::string host = request.value("host", "127.0.0.1");
    int port = request.value("port", 10300);
    try {
      sim_sender_.setEndpoint(host, port);
      RCLCPP_INFO(this->get_logger(), "Sim endpoint updated to %s:%d", host.c_str(), port);
      json result = {
        {"type", "sim_endpoint_set"}, {"success", true}, {"host", host}, {"port", port}};
      connection->sendMessage(result.dump());
    } catch (const std::exception & e) {
      json error = {{"type", "error"}, {"message", std::string("Invalid endpoint: ") + e.what()}};
      connection->sendMessage(error.dump());
    }
  }

  void broadcastToAll(const std::string & message)
  {
    // 送信はブロッキングなので、一覧のコピーだけをロック中に取る。
    // ロックを持ったまま送ると、遅いクライアント 1 台が接続の出入りまで止める
    std::vector<std::shared_ptr<WebSocketConnection>> targets;
    {
      std::lock_guard<std::mutex> lock(connections_mutex_);
      targets.assign(connections_.begin(), connections_.end());
    }
    for (const auto & connection : targets) {
      connection->sendMessage(message);
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr session_injection_sub_;
  std::vector<std::string> cached_situation_names_;
  std::string current_injection_{"HALT"};
  std::deque<std::pair<std::string, int64_t>> injection_history_;
  std::mutex injection_mutex_;
  rclcpp::Publisher<crane_msgs::msg::RobotCommand>::SharedPtr robot_test_target_pub_;
  rclcpp::AsyncParametersClient::SharedPtr local_planner_params_client_;
  rclcpp::AsyncParametersClient::SharedPtr ibis_sender_params_client_;

  // Cached world model state for move commands
  bool cached_is_yellow_{false};
  bool cached_on_positive_half_{false};
  std::mutex world_model_cache_mutex_;

  // Robot monitoring
  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr robot_feedback_sub_;
  rclcpp::Subscription<crane_msgs::msg::PingStatusArray>::SharedPtr ping_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::TimerBase::SharedPtr robot_feedback_timer_;
  LatestMessage<crane_msgs::msg::RobotFeedbackArray> robot_feedback_;

  // Control targets monitoring
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr control_targets_sub_;
  rclcpp::TimerBase::SharedPtr control_targets_timer_;
  LatestMessage<crane_msgs::msg::RobotCommands> control_targets_;

  // Latency estimation monitoring
  rclcpp::Subscription<crane_msgs::msg::LatencyEstimationArray>::SharedPtr latency_estimation_sub_;
  rclcpp::TimerBase::SharedPtr latency_estimation_timer_;
  LatestMessage<crane_msgs::msg::LatencyEstimationArray> latency_estimation_;

  // Server components
  std::thread websocket_thread_;
  int websocket_port_;
  std::atomic<bool> running_{true};
  boost::asio::io_context ws_io_context_;
  std::unique_ptr<boost::asio::ip::tcp::acceptor> ws_acceptor_;

  // WebSocket connections
  std::set<std::shared_ptr<WebSocketConnection>> connections_;
  std::mutex connections_mutex_;

  // 接続スレッドが生きている間のソケット。stopServers がこれを空になるまで待つ
  std::set<std::shared_ptr<boost::asio::ip::tcp::socket>> sockets_;
  std::mutex sockets_mutex_;
  std::condition_variable sockets_cv_;

  // Game info cache
  crane_msgs::msg::PlaySituation::SharedPtr latest_play_situation_;
  std::mutex game_info_mutex_;

  // World model throttle (10Hz)
  LatestMessage<crane_msgs::msg::WorldModel> world_model_;
  rclcpp::TimerBase::SharedPtr world_model_timer_;

  // SVG snapshot throttle (5Hz)
  LatestMessage<crane_visualization_interfaces::msg::SvgSnapshot> svg_snapshot_;
  rclcpp::TimerBase::SharedPtr svg_snapshot_timer_;

  // SVG updates coalescing (20Hz)
  std::vector<crane_visualization_interfaces::msg::SvgUpdates::SharedPtr> pending_svg_updates_;
  std::mutex svg_updates_mutex_;
  rclcpp::TimerBase::SharedPtr svg_updates_timer_;

  // Simulation control UDP sender
  SslSimulationSender sim_sender_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WebSocketDebugServer>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
