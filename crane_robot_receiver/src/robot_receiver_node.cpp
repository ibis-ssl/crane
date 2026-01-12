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
#include <crane_robot_receiver/robot_feedback_protocol.hpp>
#include <deque>
#include <format>
#include <rclcpp/rclcpp.hpp>
#include <unordered_set>

using boost::asio::ip::udp;
using crane::robot_receiver::RobotFeedback;
namespace protocol = crane::robot_receiver::protocol;

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
      // 受信タイムスタンプを記録
      receive_timestamps_.push_back(clock.now());
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

  auto getPacketFrequency() -> float
  {
    using std::chrono::operator""ms;
    auto now = clock.now();

    // 1秒より古いタイムスタンプを削除
    while (!receive_timestamps_.empty() && (now - receive_timestamps_.front()) > 1000ms) {
      receive_timestamps_.pop_front();
    }

    // 最近1秒間の受信回数を返す
    return static_cast<float>(receive_timestamps_.size());
  }

  const int robot_id;

private:
  std::unique_ptr<crane::MulticastReceiver> receiver_;

  std::vector<char> buffer_;

  RobotFeedback robot_feedback;

  rclcpp::Clock clock;

  // パケット受信頻度計算用のタイムスタンプキュー
  std::deque<rclcpp::Time> receive_timestamps_;
};

class RobotReceiverNode : public rclcpp::Node
{
public:
  explicit RobotReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("robot_receiver_node", options), clock(RCL_ROS_TIME)
  {
    crane::CraneVisualizerBuffer::activate(*this);
    publisher = create_publisher<crane_msgs::msg::RobotFeedbackArray>("/robot_feedback", 10);

    // パラメータの宣言と取得
    int max_robot_id = declare_parameter("max_robot_id", 15);
    std::string ip_base = declare_parameter("multicast_ip_base", "224.5.20");
    int port_base = declare_parameter("port_base", 50100);
    int ip_offset = declare_parameter("ip_octet_offset", 100);

    RCLCPP_INFO(get_logger(), "Listening for robot feedbacks (max_robot_id: %d)", max_robot_id);

    for (int i = 0; i <= max_robot_id; i++) {
      std::string ip = std::format("{}.{}", ip_base, i + ip_offset);
      int port = port_base + i;
      receivers.push_back(std::make_shared<RobotFeedbackReceiver>(ip, port));
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
          robot_feedback_msg.packet_frequency_hz = receiver->getPacketFrequency();
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
