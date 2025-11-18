// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <sys/socket.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msgs/msg/robot_feedback.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <format>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <unordered_set>

using boost::asio::ip::udp;

// SO_REUSEPORTソケットオプションの定義
typedef boost::asio::detail::socket_option::boolean<SOL_SOCKET, SO_REUSEPORT> reuse_port;

struct RobotInterfaceConfig
{
  std::string ip;

  int port;
};

auto makeConfig(uint8_t id) -> RobotInterfaceConfig
{
  RobotInterfaceConfig config;
  config.ip = std::format("224.5.20.{}", id + 100);
  config.port = 50100 + id;
  return config;
}

struct RobotFeedback
{
  rclcpp::Time received_stamp;

  uint8_t counter = 0;

  uint8_t kick_state = 0;

  uint8_t temperature[7] = {};

  uint16_t error_id = 0;

  uint16_t error_info = 0;

  float error_value = 0.0f;

  float motor_current[4] = {};

  uint8_t ball_detection[4] = {};

  bool ball_sensor = false;

  float_t yaw_angle = 0.0f;

  float_t diff_angle = 0.0f;

  std::array<float_t, 2> odom = {0.0f, 0.0f};

  std::array<float_t, 2> odom_speed = {0.0f, 0.0f};

  std::array<float_t, 2> mouse_odom = {0.0f, 0.0f};

  std::array<float_t, 2> mouse_vel = {0.0f, 0.0f};

  std::array<float_t, 2> voltage = {0.0f, 0.0f};

  uint8_t check_ver = 0;

  std::vector<float> values;
};

union FloatUnion {
  float f;

  std::array<char, 4> b;
};

union Uint16Union {
  uint16_t u16;

  std::array<char, 2> b;
};

// ロボットフィードバックプロトコル定数
namespace protocol
{
// バッファサイズ
constexpr size_t BUFFER_SIZE = 2048;
constexpr size_t DEBUG_VALUES_END = 128;

// バイトオフセット定義
namespace offset
{
constexpr int COUNTER = 3;

constexpr int YAW_ANGLE = 4;
constexpr int VOLTAGE_0 = 8;

constexpr int BALL_DETECTION_0 = 12;
constexpr int BALL_DETECTION_1 = 13;
constexpr int BALL_DETECTION_2 = 14;
constexpr int KICK_STATE = 15;

constexpr int ERROR_ID = 16;
constexpr int ERROR_INFO = 18;
constexpr int ERROR_VALUE = 20;

constexpr int MOTOR_CURRENT_0 = 24;
constexpr int MOTOR_CURRENT_1 = 25;
constexpr int MOTOR_CURRENT_2 = 26;
constexpr int MOTOR_CURRENT_3 = 27;

constexpr int BALL_DETECTION_3 = 28;

constexpr int TEMPERATURE_0 = 29;
constexpr int TEMPERATURE_1 = 30;
constexpr int TEMPERATURE_2 = 31;
constexpr int TEMPERATURE_3 = 32;
constexpr int TEMPERATURE_4 = 33;
constexpr int TEMPERATURE_5 = 34;
constexpr int TEMPERATURE_6 = 35;

constexpr int DIFF_ANGLE = 36;
constexpr int VOLTAGE_1 = 40;

constexpr int ODOM_X = 44;
constexpr int ODOM_Y = 48;
constexpr int ODOM_SPEED_X = 52;
constexpr int ODOM_SPEED_Y = 56;

constexpr int CHECK_VER = 60;

constexpr int MOUSE_ODOM_X = 64;
constexpr int MOUSE_ODOM_Y = 68;
constexpr int MOUSE_VEL_X = 72;
constexpr int MOUSE_VEL_Y = 76;

constexpr int DEBUG_VALUES_START = 64;
}  // namespace offset

// 定数値
constexpr float MOTOR_CURRENT_SCALE = 10.0f;
constexpr int KICK_STATE_SCALE = 10;
constexpr int FLOAT_SIZE = 4;

// バッファ読み取りヘルパー関数
inline auto readFloat(const std::vector<uint8_t> & buffer, int offset) -> float
{
  FloatUnion float_union;
  float_union.b[0] = buffer[offset];
  float_union.b[1] = buffer[offset + 1];
  float_union.b[2] = buffer[offset + 2];
  float_union.b[3] = buffer[offset + 3];
  return float_union.f;
}

inline auto readUint16(const std::vector<uint8_t> & buffer, int offset) -> uint16_t
{
  Uint16Union uint16_union;
  uint16_union.b[0] = buffer[offset];
  uint16_union.b[1] = buffer[offset + 1];
  return uint16_union.u16;
}

inline auto readByte(const std::vector<uint8_t> & buffer, int offset) -> uint8_t
{
  return buffer[offset];
}
}  // namespace protocol

class MulticastReceiver
{
public:
  MulticastReceiver(const std::string & host, const int port)
  : robot_id(port - 50100),
    socket(io_service, boost::asio::ip::udp::v4()),
    buffer(protocol::BUFFER_SIZE),
    received_size(0),
    clock(RCL_ROS_TIME)
  {
    // 初回比較時のエラー回避
    robot_feedback.received_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    boost::asio::ip::address addr = boost::asio::ip::address::from_string(host);
    if (!addr.is_multicast()) {
      throw std::runtime_error("expected multicast address");
    }

    // ソケットオプションを先に設定
    boost::system::error_code ec;

    socket.set_option(boost::asio::socket_base::reuse_address(true), ec);

    socket.set_option(reuse_port(true), ec);

    socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port), ec);
    if (ec) {
      std::cerr << "[ERROR] bind失敗: " << ec.message() << std::endl;
      throw std::runtime_error("bind failed: " + ec.message());
    } else {
      std::cout << "[DEBUG] bind成功: port=" << port << std::endl;
    }

    socket.non_blocking(true);

    // その後、全てのネットワークデバイスでマルチキャストに参加
    try {
      struct ifaddrs * interfaces = nullptr;
      struct ifaddrs * ifa = nullptr;

      // ネットワークインターフェース情報の取得
      if (getifaddrs(&interfaces) == -1) {
        throw std::runtime_error("Error: getifaddrs failed.");
      }

      int interface_count = 0;
      int success_count = 0;
      int skip_count = 0;
      std::unordered_set<std::string> joined_interfaces;  // 参加済みインターフェース名を記録

      // ネットワークインターフェースのリストを巡回
      for (ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) {
          continue;
        }

        if (ifa->ifa_addr->sa_family == AF_INET) {
          // IPv4アドレスのみ
          char ip[INET_ADDRSTRLEN];
          inet_ntop(
            AF_INET, &(reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr)->sin_addr), ip,
            INET_ADDRSTRLEN);

          interface_count++;
          std::cout << "[DEBUG] インターフェース検出: " << ifa->ifa_name << ": " << ip << std::endl;

          // 同じインターフェースで既に参加済みの場合はスキップ
          std::string if_name(ifa->ifa_name);
          if (joined_interfaces.count(if_name) > 0) {
            skip_count++;
            continue;
          }

          boost::asio::ip::detail::socket_option::multicast_request<
            IPPROTO_IP, IP_ADD_MEMBERSHIP, IPPROTO_IPV6, IPV6_JOIN_GROUP>
            join_device(addr.to_v4(), boost::asio::ip::address::from_string(ip).to_v4());

          boost::system::error_code join_ec;
          socket.set_option(join_device, join_ec);

          if (join_ec) {
            skip_count++;
          } else {
            joined_interfaces.insert(if_name);  // 参加成功したインターフェースを記録
            success_count++;
          }
        }
      }

      freeifaddrs(interfaces);  // メモリの解放
    } catch (std::exception & e) {
      std::cerr << "[ERROR] マルチキャスト設定中に例外: " << e.what() << std::endl;
    }
  }

  auto receive() -> bool
  {
    if (socket.available()) {
      boost::system::error_code error;
      received_size = socket.receive(boost::asio::buffer(buffer), 0, error);
      if (error && error != boost::asio::error::message_size) {
        throw boost::system::system_error(error);
      }
      return true;
    } else {
      return false;
    }
  }

  auto updateFeedback() -> void
  {
    using namespace protocol;

    RobotFeedback feedback;
    // 最新のデータでリセット
    feedback = robot_feedback;

    // 受信タイムスタンプを更新
    feedback.received_stamp = clock.now();

    // 基本情報
    feedback.counter = readByte(buffer, offset::COUNTER);
    feedback.yaw_angle = readFloat(buffer, offset::YAW_ANGLE);
    feedback.voltage[0] = readFloat(buffer, offset::VOLTAGE_0);

    // ボール検出
    feedback.ball_detection[0] = readByte(buffer, offset::BALL_DETECTION_0);
    feedback.ball_detection[1] = readByte(buffer, offset::BALL_DETECTION_1);
    feedback.ball_detection[2] = readByte(buffer, offset::BALL_DETECTION_2);
    feedback.ball_detection[3] = readByte(buffer, offset::BALL_DETECTION_3);
    feedback.kick_state = readByte(buffer, offset::KICK_STATE) * KICK_STATE_SCALE;

    // エラー情報
    feedback.error_id = readUint16(buffer, offset::ERROR_ID);
    feedback.error_info = readUint16(buffer, offset::ERROR_INFO);
    feedback.error_value = readFloat(buffer, offset::ERROR_VALUE);

    // モーター電流
    feedback.motor_current[0] = readByte(buffer, offset::MOTOR_CURRENT_0) / MOTOR_CURRENT_SCALE;
    feedback.motor_current[1] = readByte(buffer, offset::MOTOR_CURRENT_1) / MOTOR_CURRENT_SCALE;
    feedback.motor_current[2] = readByte(buffer, offset::MOTOR_CURRENT_2) / MOTOR_CURRENT_SCALE;
    feedback.motor_current[3] = readByte(buffer, offset::MOTOR_CURRENT_3) / MOTOR_CURRENT_SCALE;

    // 温度
    feedback.temperature[0] = readByte(buffer, offset::TEMPERATURE_0);
    feedback.temperature[1] = readByte(buffer, offset::TEMPERATURE_1);
    feedback.temperature[2] = readByte(buffer, offset::TEMPERATURE_2);
    feedback.temperature[3] = readByte(buffer, offset::TEMPERATURE_3);
    feedback.temperature[4] = readByte(buffer, offset::TEMPERATURE_4);
    feedback.temperature[5] = readByte(buffer, offset::TEMPERATURE_5);
    feedback.temperature[6] = readByte(buffer, offset::TEMPERATURE_6);

    // 角度と電圧
    feedback.diff_angle = readFloat(buffer, offset::DIFF_ANGLE);
    feedback.voltage[1] = readFloat(buffer, offset::VOLTAGE_1);

    // オドメトリ
    feedback.odom[0] = readFloat(buffer, offset::ODOM_X);
    feedback.odom[1] = readFloat(buffer, offset::ODOM_Y);
    feedback.odom_speed[0] = readFloat(buffer, offset::ODOM_SPEED_X);
    feedback.odom_speed[1] = readFloat(buffer, offset::ODOM_SPEED_Y);

    feedback.check_ver = readByte(buffer, offset::CHECK_VER);

    // マウスセンサー
    feedback.mouse_odom[0] = readFloat(buffer, offset::MOUSE_ODOM_X);
    feedback.mouse_odom[1] = readFloat(buffer, offset::MOUSE_ODOM_Y);
    feedback.mouse_vel[0] = readFloat(buffer, offset::MOUSE_VEL_X);
    feedback.mouse_vel[1] = readFloat(buffer, offset::MOUSE_VEL_Y);

    // デバッグ値
    feedback.values.clear();
    for (size_t i = offset::DEBUG_VALUES_START; i < DEBUG_VALUES_END - FLOAT_SIZE;
         i += FLOAT_SIZE) {
      feedback.values.push_back(readFloat(buffer, static_cast<int>(i)));
    }

    robot_feedback = feedback;
  }

  auto getFeedback() const -> RobotFeedback { return robot_feedback; }

  const int robot_id;

private:
  boost::asio::io_service io_service;

  boost::asio::ip::udp::socket socket;

  std::vector<uint8_t> buffer;

  size_t received_size;

  RobotFeedback robot_feedback;

  rclcpp::Clock clock;
};

class RobotReceiverNode : public rclcpp::Node
{
public:
  explicit RobotReceiverNode(uint8_t robot_num = 10)
  : rclcpp::Node("robot_receiver_node"), clock(RCL_ROS_TIME)
  {
    crane::CraneVisualizerBuffer::activate(*this);
    publisher = create_publisher<crane_msgs::msg::RobotFeedbackArray>("/robot_feedback", 10);

    for (int i = 0; i < robot_num; i++) {
      auto config = makeConfig(i);
      receivers.push_back(std::make_shared<MulticastReceiver>(config.ip, config.port));
    }

    using std::chrono::operator""ms;
    timer = rclcpp::create_timer(this, get_clock(), 10ms, [&]() {
      crane_msgs::msg::RobotFeedbackArray msg;

      auto now = clock.now();
      for (auto & receiver : receivers) {
        while (receiver->receive()) {
          receiver->updateFeedback();
        }

        // 古いデータは入れない(100msより古いデータはVisionより価値が薄い可能性が高い)
        if (auto robot_feedback = receiver->getFeedback();
            (now - robot_feedback.received_stamp) < 100ms) {
          crane_msgs::msg::RobotFeedback robot_feedback_msg;
          robot_feedback_msg.received_stamp = robot_feedback.received_stamp;
          robot_feedback_msg.robot_id = receiver->robot_id;
          robot_feedback_msg.counter = robot_feedback.counter;
          robot_feedback_msg.kick_state = robot_feedback.kick_state;
          for (auto temperature : robot_feedback.temperature) {
            robot_feedback_msg.temperatures.push_back(temperature);
          }

          robot_feedback_msg.error_id = robot_feedback.error_id;
          robot_feedback_msg.error_info = robot_feedback.error_info;

          robot_feedback_msg.error_value = robot_feedback.error_value;

          for (auto motor_current : robot_feedback.motor_current) {
            robot_feedback_msg.motor_current.push_back(motor_current);
          }
          for (auto ball_detection : robot_feedback.ball_detection) {
            robot_feedback_msg.ball_detection.push_back(ball_detection);
          }
          robot_feedback_msg.ball_sensor =
            static_cast<bool>(robot_feedback_msg.ball_detection[0] == 1);
          robot_feedback_msg.yaw_angle = robot_feedback.yaw_angle;
          robot_feedback_msg.diff_angle = robot_feedback.diff_angle;
          for (auto odom : robot_feedback.odom) {
            robot_feedback_msg.odom.push_back(odom);
          }
          for (auto odom_speed : robot_feedback.odom_speed) {
            robot_feedback_msg.odom_speed.push_back(odom_speed);
          }
          for (auto mouse_odom : robot_feedback.mouse_odom) {
            robot_feedback_msg.mouse_odom.push_back(mouse_odom);
          }

          for (auto mouse_vel : robot_feedback.mouse_vel) {
            robot_feedback_msg.mouse_vel.push_back(mouse_vel);
          }
          for (auto voltage : robot_feedback.voltage) {
            robot_feedback_msg.voltage.push_back(voltage);
          }
          robot_feedback_msg.check_ver = robot_feedback.check_ver;

          for (const auto & value : robot_feedback.values) {
            robot_feedback_msg.values.push_back(value);
          }
          msg.feedback.push_back(robot_feedback_msg);
        }
      }
      publisher->publish(msg);
      visualizer->flush();
      crane::CraneVisualizerBuffer::publish();
    });
  }

  rclcpp::TimerBase::SharedPtr timer;

  std::vector<std::shared_ptr<MulticastReceiver>> receivers;

  rclcpp::Publisher<crane_msgs::msg::RobotFeedbackArray>::SharedPtr publisher;

  crane::VisualizerMessageBuilder::SharedPtr visualizer =
    std::make_unique<crane::VisualizerMessageBuilder>("robot_receiver");

  rclcpp::Clock clock;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
