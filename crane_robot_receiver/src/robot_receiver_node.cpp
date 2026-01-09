// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <crane_comm/multicast.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msgs/msg/robot_feedback.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <format>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <unordered_set>

using boost::asio::ip::udp;

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

// std::vector<char> 版のオーバーロード（crane::MulticastReceiver用）
inline auto readFloat(const std::vector<char> & buffer, int offset) -> float
{
  return readFloat(reinterpret_cast<const std::vector<uint8_t> &>(buffer), offset);
}

inline auto readUint16(const std::vector<char> & buffer, int offset) -> uint16_t
{
  return readUint16(reinterpret_cast<const std::vector<uint8_t> &>(buffer), offset);
}

inline auto readByte(const std::vector<char> & buffer, int offset) -> uint8_t
{
  return readByte(reinterpret_cast<const std::vector<uint8_t> &>(buffer), offset);
}
}  // namespace protocol

class RobotFeedbackReceiver
{
public:
  RobotFeedbackReceiver(const std::string & host, const int port)
  : robot_id(port - 50100),
    receiver_(std::make_unique<crane::MulticastReceiver>(host, port)),
    buffer_(protocol::BUFFER_SIZE),
    clock(RCL_ROS_TIME)
  {
    // 初回比較時のエラー回避
    robot_feedback.received_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  auto receive() -> bool
  {
    if (receiver_->available()) {
      receiver_->receive(buffer_);
      return true;
    }
    return false;
  }

  auto updateFeedback() -> void
  {
    RobotFeedback feedback;
    // 最新のデータでリセット
    feedback = robot_feedback;

    // 受信タイムスタンプを更新
    feedback.received_stamp = clock.now();

    // 基本情報
    feedback.counter = protocol::readByte(buffer_, protocol::offset::COUNTER);
    feedback.yaw_angle = protocol::readFloat(buffer_, protocol::offset::YAW_ANGLE);
    feedback.voltage[0] = protocol::readFloat(buffer_, protocol::offset::VOLTAGE_0);

    // ボール検出
    feedback.ball_detection[0] = protocol::readByte(buffer_, protocol::offset::BALL_DETECTION_0);
    feedback.ball_detection[1] = protocol::readByte(buffer_, protocol::offset::BALL_DETECTION_1);
    feedback.ball_detection[2] = protocol::readByte(buffer_, protocol::offset::BALL_DETECTION_2);
    feedback.ball_detection[3] = protocol::readByte(buffer_, protocol::offset::BALL_DETECTION_3);
    feedback.kick_state =
      protocol::readByte(buffer_, protocol::offset::KICK_STATE) * protocol::KICK_STATE_SCALE;

    // エラー情報
    feedback.error_id = protocol::readUint16(buffer_, protocol::offset::ERROR_ID);
    feedback.error_info = protocol::readUint16(buffer_, protocol::offset::ERROR_INFO);
    feedback.error_value = protocol::readFloat(buffer_, protocol::offset::ERROR_VALUE);

    // モーター電流
    feedback.motor_current[0] = protocol::readByte(buffer_, protocol::offset::MOTOR_CURRENT_0) /
                                protocol::MOTOR_CURRENT_SCALE;
    feedback.motor_current[1] = protocol::readByte(buffer_, protocol::offset::MOTOR_CURRENT_1) /
                                protocol::MOTOR_CURRENT_SCALE;
    feedback.motor_current[2] = protocol::readByte(buffer_, protocol::offset::MOTOR_CURRENT_2) /
                                protocol::MOTOR_CURRENT_SCALE;
    feedback.motor_current[3] = protocol::readByte(buffer_, protocol::offset::MOTOR_CURRENT_3) /
                                protocol::MOTOR_CURRENT_SCALE;

    // 温度
    feedback.temperature[0] = protocol::readByte(buffer_, protocol::offset::TEMPERATURE_0);
    feedback.temperature[1] = protocol::readByte(buffer_, protocol::offset::TEMPERATURE_1);
    feedback.temperature[2] = protocol::readByte(buffer_, protocol::offset::TEMPERATURE_2);
    feedback.temperature[3] = protocol::readByte(buffer_, protocol::offset::TEMPERATURE_3);
    feedback.temperature[4] = protocol::readByte(buffer_, protocol::offset::TEMPERATURE_4);
    feedback.temperature[5] = protocol::readByte(buffer_, protocol::offset::TEMPERATURE_5);
    feedback.temperature[6] = protocol::readByte(buffer_, protocol::offset::TEMPERATURE_6);

    // 角度と電圧
    feedback.diff_angle = protocol::readFloat(buffer_, protocol::offset::DIFF_ANGLE);
    feedback.voltage[1] = protocol::readFloat(buffer_, protocol::offset::VOLTAGE_1);

    // オドメトリ
    feedback.odom[0] = protocol::readFloat(buffer_, protocol::offset::ODOM_X);
    feedback.odom[1] = protocol::readFloat(buffer_, protocol::offset::ODOM_Y);
    feedback.odom_speed[0] = protocol::readFloat(buffer_, protocol::offset::ODOM_SPEED_X);
    feedback.odom_speed[1] = protocol::readFloat(buffer_, protocol::offset::ODOM_SPEED_Y);

    feedback.check_ver = protocol::readByte(buffer_, protocol::offset::CHECK_VER);

    // マウスセンサー
    feedback.mouse_odom[0] = protocol::readFloat(buffer_, protocol::offset::MOUSE_ODOM_X);
    feedback.mouse_odom[1] = protocol::readFloat(buffer_, protocol::offset::MOUSE_ODOM_Y);
    feedback.mouse_vel[0] = protocol::readFloat(buffer_, protocol::offset::MOUSE_VEL_X);
    feedback.mouse_vel[1] = protocol::readFloat(buffer_, protocol::offset::MOUSE_VEL_Y);

    // デバッグ値
    feedback.values.clear();
    for (size_t i = protocol::offset::DEBUG_VALUES_START;
         i < protocol::DEBUG_VALUES_END - protocol::FLOAT_SIZE; i += protocol::FLOAT_SIZE) {
      feedback.values.push_back(protocol::readFloat(buffer_, static_cast<int>(i)));
    }

    robot_feedback = feedback;
  }

  auto getFeedback() const -> RobotFeedback { return robot_feedback; }

  const int robot_id;

private:
  std::unique_ptr<crane::MulticastReceiver> receiver_;

  std::vector<char> buffer_;

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
      receivers.push_back(std::make_shared<RobotFeedbackReceiver>(config.ip, config.port));
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

  std::vector<std::shared_ptr<RobotFeedbackReceiver>> receivers;

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
