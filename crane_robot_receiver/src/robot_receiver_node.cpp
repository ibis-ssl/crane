// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <crane_comm/unicast.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msgs/msg/robot_feedback.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_robot_receiver/robot_feedback_protocol.hpp>
#include <deque>
#include <format>
#include <limits>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <unordered_set>

namespace asio = boost::asio;
using crane::robot_receiver::RobotFeedback;
namespace protocol = crane::robot_receiver::protocol;

class RobotFeedbackReceiver
{
public:
  RobotFeedbackReceiver(
    asio::io_context & io_ctx, const std::string & host, const int port, float alpha)
  : robot_id(port - 50100),
    alpha_(alpha),
    async_receiver_(
      std::make_unique<crane::AsyncUdpReceiver>(io_ctx, host, port, protocol::BUFFER_SIZE)),
    clock(RCL_ROS_TIME)
  {
    async_receiver_->startReceive(
      [this](const std::vector<char> & buf, size_t size) { onReceive(buf, size); });
  }

  // asioスレッドから呼ばれる
  void onReceive(const std::vector<char> & buf, size_t /*size*/)
  {
    auto stamp = clock.now();
    RobotFeedback new_packet = parseBuffer(buf, stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!master_feedback_) {
      master_feedback_ = new_packet;
    } else {
      blendFeedback(master_feedback_.value(), new_packet, alpha_);
    }
    receive_timestamps_.push_back(stamp);
  }

  // ROS2タイマーから呼ばれる
  auto getLatestFeedback() -> std::optional<RobotFeedback>
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return master_feedback_;
  }

  struct IntervalStats
  {
    float mean_ms = 0.f;
    float min_ms = 0.f;
    float max_ms = 0.f;
    float stddev_ms = 0.f;
  };

  struct ReceiverStats
  {
    float packet_frequency_hz = 0.f;
    IntervalStats interval;
  };

  // 周波数と受信間隔統計を1回のロックでまとめて取得する
  auto getStats() -> ReceiverStats
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pruneTimestampsLocked();
    ReceiverStats result;
    result.packet_frequency_hz = static_cast<float>(receive_timestamps_.size());
    if (receive_timestamps_.size() >= 2) {
      result.interval = computeIntervalStatsLocked();
    }
    return result;
  }

  const int robot_id;

private:
  auto parseBuffer(const std::vector<char> & buf, rclcpp::Time stamp) -> RobotFeedback
  {
    RobotFeedback feedback;
    feedback.received_stamp = stamp;

    // 基本情報
    feedback.counter = protocol::readByte(buf, protocol::offset::COUNTER);
    feedback.yaw_angle = protocol::readFloat(buf, protocol::offset::YAW_ANGLE);
    feedback.voltage[0] = protocol::readFloat(buf, protocol::offset::VOLTAGE_0);

    // ボール検出
    feedback.ball_detection[0] = protocol::readByte(buf, protocol::offset::BALL_DETECTION_0);
    feedback.ball_detection[1] = protocol::readByte(buf, protocol::offset::BALL_DETECTION_1);
    feedback.ball_detection[2] = protocol::readByte(buf, protocol::offset::BALL_DETECTION_2);
    feedback.ball_detection[3] = protocol::readByte(buf, protocol::offset::BALL_DETECTION_3);
    feedback.kick_state =
      protocol::readByte(buf, protocol::offset::KICK_STATE) * protocol::KICK_STATE_SCALE;

    // エラー情報
    feedback.error_id = protocol::readUint16(buf, protocol::offset::ERROR_ID);
    feedback.error_info = protocol::readUint16(buf, protocol::offset::ERROR_INFO);
    feedback.error_value = protocol::readFloat(buf, protocol::offset::ERROR_VALUE);

    // モーター電流
    feedback.motor_current[0] =
      protocol::readByte(buf, protocol::offset::MOTOR_CURRENT_0) / protocol::MOTOR_CURRENT_SCALE;
    feedback.motor_current[1] =
      protocol::readByte(buf, protocol::offset::MOTOR_CURRENT_1) / protocol::MOTOR_CURRENT_SCALE;
    feedback.motor_current[2] =
      protocol::readByte(buf, protocol::offset::MOTOR_CURRENT_2) / protocol::MOTOR_CURRENT_SCALE;
    feedback.motor_current[3] =
      protocol::readByte(buf, protocol::offset::MOTOR_CURRENT_3) / protocol::MOTOR_CURRENT_SCALE;

    // 温度
    feedback.temperature[0] = protocol::readByte(buf, protocol::offset::TEMPERATURE_0);
    feedback.temperature[1] = protocol::readByte(buf, protocol::offset::TEMPERATURE_1);
    feedback.temperature[2] = protocol::readByte(buf, protocol::offset::TEMPERATURE_2);
    feedback.temperature[3] = protocol::readByte(buf, protocol::offset::TEMPERATURE_3);
    feedback.temperature[4] = protocol::readByte(buf, protocol::offset::TEMPERATURE_4);
    feedback.temperature[5] = protocol::readByte(buf, protocol::offset::TEMPERATURE_5);
    feedback.temperature[6] = protocol::readByte(buf, protocol::offset::TEMPERATURE_6);

    // 角度と電圧
    feedback.diff_angle = protocol::readFloat(buf, protocol::offset::DIFF_ANGLE);
    feedback.voltage[1] = protocol::readFloat(buf, protocol::offset::VOLTAGE_1);

    // オドメトリ
    feedback.odom[0] = protocol::readFloat(buf, protocol::offset::ODOM_X);
    feedback.odom[1] = protocol::readFloat(buf, protocol::offset::ODOM_Y);
    feedback.odom_speed[0] = protocol::readFloat(buf, protocol::offset::ODOM_SPEED_X);
    feedback.odom_speed[1] = protocol::readFloat(buf, protocol::offset::ODOM_SPEED_Y);

    feedback.check_ver = protocol::readByte(buf, protocol::offset::CHECK_VER);

    // マウスセンサー
    feedback.mouse_odom[0] = protocol::readFloat(buf, protocol::offset::MOUSE_ODOM_X);
    feedback.mouse_odom[1] = protocol::readFloat(buf, protocol::offset::MOUSE_ODOM_Y);
    feedback.mouse_vel[0] = protocol::readFloat(buf, protocol::offset::MOUSE_VEL_X);
    feedback.mouse_vel[1] = protocol::readFloat(buf, protocol::offset::MOUSE_VEL_Y);

    // デバッグ値
    feedback.values.clear();
    for (size_t i = protocol::offset::DEBUG_VALUES_START;
         i < protocol::DEBUG_VALUES_END - protocol::FLOAT_SIZE; i += protocol::FLOAT_SIZE) {
      feedback.values.push_back(protocol::readFloat(buf, static_cast<int>(i)));
    }

    return feedback;
  }

  // mutex_ を保持した状態で呼ぶこと
  void pruneTimestampsLocked()
  {
    using std::chrono::operator""ms;
    auto now = clock.now();
    while (!receive_timestamps_.empty() && (now - receive_timestamps_.front()) > 1000ms) {
      receive_timestamps_.pop_front();
    }
  }

  // mutex_ を保持した状態で呼ぶこと。サイズが2以上であることを確認してから呼ぶ
  auto computeIntervalStatsLocked() -> IntervalStats
  {
    const size_t n = receive_timestamps_.size() - 1;
    float sum = 0.f;
    float min_val = std::numeric_limits<float>::max();
    float max_val = std::numeric_limits<float>::lowest();
    for (size_t i = 1; i <= n; ++i) {
      float dt = static_cast<float>(
        (receive_timestamps_[i] - receive_timestamps_[i - 1]).nanoseconds() * 1e-6);
      sum += dt;
      if (dt < min_val) min_val = dt;
      if (dt > max_val) max_val = dt;
    }
    float mean = sum / static_cast<float>(n);
    float var = 0.f;
    for (size_t i = 1; i <= n; ++i) {
      float dt = static_cast<float>(
        (receive_timestamps_[i] - receive_timestamps_[i - 1]).nanoseconds() * 1e-6);
      float d = dt - mean;
      var += d * d;
    }
    return {mean, min_val, max_val, std::sqrt(var / static_cast<float>(n))};
  }

  static void blendFeedback(RobotFeedback & master, const RobotFeedback & incoming, float alpha)
  {
    // タイムスタンプ・離散値: 直接上書き
    master.received_stamp = incoming.received_stamp;
    master.counter = incoming.counter;
    master.kick_state = incoming.kick_state;
    master.error_id = incoming.error_id;
    master.error_info = incoming.error_info;
    master.error_value = incoming.error_value;
    master.ball_sensor = incoming.ball_sensor;
    master.check_ver = incoming.check_ver;
    master.values = incoming.values;
    std::copy(
      std::begin(incoming.ball_detection), std::end(incoming.ball_detection),
      std::begin(master.ball_detection));
    std::copy(
      std::begin(incoming.temperature), std::end(incoming.temperature),
      std::begin(master.temperature));

    // 連続値: EMAブレンド (master = alpha * incoming + (1 - alpha) * master)
    // TODO(ibis-ssl): yaw_angle/diff_angleは±π境界でラップアラウンドが起きる場合がある。
    //       問題発生時はcircular EMAへの変更を検討すること。
    const float inv_alpha = 1.0f - alpha;
    master.yaw_angle = alpha * incoming.yaw_angle + inv_alpha * master.yaw_angle;
    master.diff_angle = alpha * incoming.diff_angle + inv_alpha * master.diff_angle;
    for (int i = 0; i < 4; ++i) {
      master.motor_current[i] =
        alpha * incoming.motor_current[i] + inv_alpha * master.motor_current[i];
    }
    auto blend_array2 = [alpha, inv_alpha](
                          std::array<float, 2> & dst, const std::array<float, 2> & src) {
      for (int i = 0; i < 2; ++i) {
        dst[i] = alpha * src[i] + inv_alpha * dst[i];
      }
    };
    blend_array2(master.odom, incoming.odom);
    blend_array2(master.odom_speed, incoming.odom_speed);
    blend_array2(master.mouse_odom, incoming.mouse_odom);
    blend_array2(master.mouse_vel, incoming.mouse_vel);
    blend_array2(master.voltage, incoming.voltage);
  }

  float alpha_;
  std::unique_ptr<crane::AsyncUdpReceiver> async_receiver_;
  std::mutex mutex_;
  std::optional<RobotFeedback> master_feedback_;
  std::deque<rclcpp::Time> receive_timestamps_;
  rclcpp::Clock clock;
};

class RobotReceiverNode : public rclcpp::Node
{
public:
  explicit RobotReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("robot_receiver_node", options),
    work_guard_(asio::make_work_guard(io_context_)),
    clock(RCL_ROS_TIME)
  {
    crane::CraneVisualizerBuffer::activate(*this);
    publisher = create_publisher<crane_msgs::msg::RobotFeedbackArray>("/robot_feedback", 10);

    // パラメータの宣言と取得
    int max_robot_id = declare_parameter("max_robot_id", 15);
    bool sim_mode = declare_parameter("sim_mode", false);
    std::string ip_base = declare_parameter("multicast_ip_base", "224.5.20");
    int port_base = declare_parameter("port_base", 50100);
    int ip_offset = declare_parameter("ip_octet_offset", 100);
    // EMAブレンド係数: 0.0=全く追従しない, 1.0=即座に追従(従来動作と同等)
    float feedback_alpha = static_cast<float>(declare_parameter("feedback_alpha", 0.1));

    RCLCPP_INFO(
      get_logger(), "Listening for robot feedbacks (max_robot_id: %d, sim_mode: %s)", max_robot_id,
      sim_mode ? "true" : "false");

    for (int i = 0; i <= max_robot_id; i++) {
      std::string ip;
      if (sim_mode) {
        ip = "127.0.0.1";
      } else {
        ip = std::format("{}.{}", ip_base, i + ip_offset);
      }
      int port = port_base + i;
      receivers.push_back(
        std::make_shared<RobotFeedbackReceiver>(io_context_, ip, port, feedback_alpha));
    }

    // asioイベントループを専用スレッドで開始
    io_thread_ = std::thread([this]() { io_context_.run(); });

    using std::chrono::operator""ms;
    using std::chrono::operator""s;
    log_timer = rclcpp::create_timer(this, get_clock(), 5s, [&]() {
      for (auto & receiver : receivers) {
        auto s = receiver->getStats();
        if (s.packet_frequency_hz <= 0.f) continue;
        RCLCPP_INFO(
          get_logger(),
          "[Robot %d] interval: mean=%.1fms, min=%.1fms, max=%.1fms, stddev=%.1fms, freq=%.1fHz",
          receiver->robot_id, s.interval.mean_ms, s.interval.min_ms, s.interval.max_ms,
          s.interval.stddev_ms, s.packet_frequency_hz);
      }
    });

    timer = rclcpp::create_timer(this, get_clock(), 10ms, [&]() {
      crane_msgs::msg::RobotFeedbackArray msg;

      auto now = clock.now();
      for (auto & receiver : receivers) {
        auto robot_feedback = receiver->getLatestFeedback();
        if (!robot_feedback) continue;

        // 古いデータは入れない(100msより古いデータはVisionより価値が薄い可能性が高い)
        using std::chrono::operator""ms;
        if ((now - robot_feedback->received_stamp) >= 100ms) continue;

        crane_msgs::msg::RobotFeedback robot_feedback_msg;
        robot_feedback_msg.received_stamp = robot_feedback->received_stamp;
        robot_feedback_msg.robot_id = receiver->robot_id;
        {
          auto s = receiver->getStats();
          robot_feedback_msg.packet_frequency_hz = s.packet_frequency_hz;
          robot_feedback_msg.receive_interval_mean_ms = s.interval.mean_ms;
          robot_feedback_msg.receive_interval_min_ms = s.interval.min_ms;
          robot_feedback_msg.receive_interval_max_ms = s.interval.max_ms;
          robot_feedback_msg.receive_interval_stddev_ms = s.interval.stddev_ms;
        }
        robot_feedback_msg.counter = robot_feedback->counter;
        robot_feedback_msg.kick_state = robot_feedback->kick_state;
        robot_feedback_msg.temperatures.assign(
          std::begin(robot_feedback->temperature), std::end(robot_feedback->temperature));

        robot_feedback_msg.error_id = robot_feedback->error_id;
        robot_feedback_msg.error_info = robot_feedback->error_info;
        robot_feedback_msg.error_value = robot_feedback->error_value;

        robot_feedback_msg.motor_current.assign(
          std::begin(robot_feedback->motor_current), std::end(robot_feedback->motor_current));
        robot_feedback_msg.ball_detection.assign(
          std::begin(robot_feedback->ball_detection), std::end(robot_feedback->ball_detection));
        robot_feedback_msg.ball_sensor = (robot_feedback_msg.ball_detection[0] == 1);
        robot_feedback_msg.yaw_angle = robot_feedback->yaw_angle;
        robot_feedback_msg.diff_angle = robot_feedback->diff_angle;
        robot_feedback_msg.odom.assign(robot_feedback->odom.begin(), robot_feedback->odom.end());
        robot_feedback_msg.odom_speed.assign(
          robot_feedback->odom_speed.begin(), robot_feedback->odom_speed.end());
        robot_feedback_msg.mouse_odom.assign(
          robot_feedback->mouse_odom.begin(), robot_feedback->mouse_odom.end());
        robot_feedback_msg.mouse_vel.assign(
          robot_feedback->mouse_vel.begin(), robot_feedback->mouse_vel.end());
        robot_feedback_msg.voltage.assign(
          robot_feedback->voltage.begin(), robot_feedback->voltage.end());
        robot_feedback_msg.check_ver = robot_feedback->check_ver;
        robot_feedback_msg.values = robot_feedback->values;
        msg.feedback.push_back(robot_feedback_msg);
      }
      publisher->publish(msg);
      visualizer->flush();
      crane::CraneVisualizerBuffer::publish();
    });
  }

  ~RobotReceiverNode()
  {
    work_guard_.reset();
    io_context_.stop();
    if (io_thread_.joinable()) io_thread_.join();
  }

  std::vector<std::shared_ptr<RobotFeedbackReceiver>> receivers;

  rclcpp::Publisher<crane_msgs::msg::RobotFeedbackArray>::SharedPtr publisher;

  crane::VisualizerMessageBuilder::SharedPtr visualizer =
    std::make_shared<crane::VisualizerMessageBuilder>("receiver/feedback");

private:
  asio::io_context io_context_;
  asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
  std::thread io_thread_;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr log_timer;

  rclcpp::Clock clock;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
