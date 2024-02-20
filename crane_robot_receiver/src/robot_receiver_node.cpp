// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <crane_msg_wrappers/consai_visualizer_wrapper.hpp>
#include <crane_msgs/msg/robot_feedback.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using boost::asio::ip::udp;

struct RobotInterfaceConfig
{
  std::string ip;
  int port;
};

auto makeConfig(uint8_t id) -> RobotInterfaceConfig
{
  RobotInterfaceConfig config;
  config.ip = "224.5.20." + std::to_string(id + 100);
  config.port = 50100 + id;
  return config;
}

struct RobotFeedback
{
  uint8_t head[2];
  uint8_t counter, return_counter;

  uint8_t kick_state;

  uint8_t temperature[7];
  uint8_t error_info[8];
  int8_t motor_current[4];
  uint8_t ball_detection[4];

  float_t yaw_angle, diff_angle;
  float_t odom[2], odom_speed[2], mouse_raw[2], voltage[2];
};

static constexpr int TX_BUF_SIZE_ETHER = 128;

typedef union {
  uint8_t buf[TX_BUF_SIZE_ETHER];
  RobotFeedback data;
} RobotInfoUnion;

union FloatUnion {
  float f;
  char b[4];
};

class MulticastReceiver
{
public:
  MulticastReceiver(const std::string & host, const int port)
  : socket(io_service, boost::asio::ip::udp::v4()),
    buffer(2048),
    robot_id(port - 50100)
  {
    boost::asio::ip::address addr = boost::asio::ip::address::from_string(host);
    if (!addr.is_multicast()) {
      throw std::runtime_error("expected multicast address");
    }

    socket.set_option(boost::asio::socket_base::reuse_address(true));
    socket.set_option(boost::asio::ip::multicast::join_group(addr.to_v4()));
    socket.bind(
      boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));
    socket.non_blocking(true);
  }

  bool receive()
  {
    if (socket.available()) {
      boost::system::error_code error;
      received_size = socket.receive(boost::asio::buffer(buffer), 0, error);
      if (error && error != boost::asio::error::message_size) {
        throw boost::system::system_error(error);
        return false;
      }
      return true;
    } else {
      return false;
    }
  }

  void updateFeedback()
  {
    FloatUnion float_union;
    RobotInfoUnion robot_info_union;
    robot_info_union.data = robot_feedback;

    // 0,1byte目は識別子みたいな感じ
    auto header = buffer[2];
    switch (header) {
      case 10: {
        robot_info_union.data.counter = buffer[3];
        {
          float_union.b[0] = buffer[4];
          float_union.b[1] = buffer[5];
          float_union.b[2] = buffer[6];
          float_union.b[3] = buffer[7];
          robot_info_union.data.yaw_angle = float_union.f;
        }
        {
          float_union.b[0] = buffer[8];
          float_union.b[1] = buffer[9];
          float_union.b[2] = buffer[10];
          float_union.b[3] = buffer[11];
          robot_info_union.data.diff_angle = float_union.f;
        }
        robot_info_union.data.ball_detection[0] = buffer[12];
        robot_info_union.data.ball_detection[1] = buffer[13];
        robot_info_union.data.ball_detection[2] = buffer[14];
        robot_info_union.data.ball_detection[3] = buffer[15];
      } break;
      case 11: {
        robot_info_union.data.error_info[0] = buffer[4];
        robot_info_union.data.error_info[1] = buffer[5];
        robot_info_union.data.error_info[2] = buffer[6];
        robot_info_union.data.error_info[3] = buffer[7];
        robot_info_union.data.error_info[4] = buffer[8];
        robot_info_union.data.error_info[5] = buffer[9];
        robot_info_union.data.error_info[6] = buffer[10];
        robot_info_union.data.error_info[7] = buffer[11];
        robot_info_union.data.motor_current[0] = buffer[12];
        robot_info_union.data.motor_current[1] = buffer[13];
        robot_info_union.data.motor_current[2] = buffer[14];
        robot_info_union.data.motor_current[3] = buffer[15];
      } break;
      case 12: {
        robot_info_union.data.kick_state = buffer[4] * 10;
        robot_info_union.data.temperature[0] = buffer[5];
        robot_info_union.data.temperature[1] = buffer[6];
        robot_info_union.data.temperature[2] = buffer[7];
        robot_info_union.data.temperature[3] = buffer[8];
        robot_info_union.data.temperature[4] = buffer[9];
        robot_info_union.data.temperature[5] = buffer[10];
        robot_info_union.data.temperature[6] = buffer[11];
        {
          float_union.b[0] = buffer[12];
          float_union.b[1] = buffer[13];
          float_union.b[2] = buffer[14];
          float_union.b[3] = buffer[15];
          robot_info_union.data.voltage[0] = float_union.f;
        }
      } break;
      case 13: {
        {
          float_union.b[0] = buffer[4];
          float_union.b[1] = buffer[5];
          float_union.b[2] = buffer[6];
          float_union.b[3] = buffer[7];
          robot_info_union.data.voltage[1] = float_union.f;
        }
        {
          float_union.b[0] = buffer[8];
          float_union.b[1] = buffer[9];
          float_union.b[2] = buffer[10];
          float_union.b[3] = buffer[11];
          robot_info_union.data.odom[0] = float_union.f * 1000;
        }
        {
          float_union.b[0] = buffer[12];
          float_union.b[1] = buffer[13];
          float_union.b[2] = buffer[14];
          float_union.b[3] = buffer[15];
          robot_info_union.data.odom[1] = float_union.f * 1000;
        }
      } break;
      case 14: {
        robot_info_union.data.return_counter = buffer[3];
        {
          float_union.b[0] = buffer[4];
          float_union.b[1] = buffer[5];
          float_union.b[2] = buffer[6];
          float_union.b[3] = buffer[7];
          robot_info_union.data.odom_speed[0] = float_union.f;
        }
        {
          float_union.b[0] = buffer[8];
          float_union.b[1] = buffer[9];
          float_union.b[2] = buffer[10];
          float_union.b[3] = buffer[11];
          robot_info_union.data.odom_speed[1] = float_union.f;
        }
      } break;
      default:
        break;
    }
    robot_feedback = robot_info_union.data;
  }

  RobotFeedback getFeedback() { return robot_feedback; }

  const int robot_id;

private:
  boost::asio::io_service io_service;

  boost::asio::ip::udp::socket socket;

  std::vector<char> buffer;

  size_t received_size;

  RobotFeedback robot_feedback;
};

class RobotReceiverNode : public rclcpp::Node
{
public:
  explicit RobotReceiverNode(uint8_t robot_num = 10)
  : rclcpp::Node("robot_receiver_node"),
    consai_visualizer_wrapper(*this, "robot_feedback")
  {
    publisher = create_publisher<crane_msgs::msg::RobotFeedbackArray>(
      "/robot_feedback", 10);

    for (int i = 0; i < robot_num; i++) {
      auto config = makeConfig(i);
      receivers.push_back(
        std::make_shared<MulticastReceiver>(config.ip, config.port));
    }

    using std::chrono_literals::ms;
    timer = create_wall_timer(10ms, [&]() {
      crane_msgs::msg::RobotFeedbackArray msg;
      for (auto & receiver : receivers) {
        if (receiver->receive()) {
          receiver->updateFeedback();
        }
        auto robot_feedback = receiver->getFeedback();
        crane_msgs::msg::RobotFeedback robot_feedback_msg;
        robot_feedback_msg.robot_id = receiver->robot_id;
        robot_feedback_msg.counter = robot_feedback.counter;
        robot_feedback_msg.return_counter = robot_feedback.return_counter;
        robot_feedback_msg.kick_state = robot_feedback.kick_state;
        for (auto temperature : robot_feedback.temperature) {
          robot_feedback_msg.temperatures.push_back(temperature);
        }
        for (auto error_info : robot_feedback.error_info) {
          robot_feedback_msg.error_info.push_back(error_info);
        }

        for (auto motor_current : robot_feedback.motor_current) {
          robot_feedback_msg.motor_current.push_back(motor_current);
        }
        for (auto ball_detection : robot_feedback.ball_detection) {
          robot_feedback_msg.ball_detection.push_back(ball_detection);
        }
        robot_feedback_msg.yaw_angle = robot_feedback.yaw_angle;
        robot_feedback_msg.diff_angle = robot_feedback.diff_angle;
        for (auto odom : robot_feedback.odom) {
          robot_feedback_msg.odom.push_back(odom);
        }
        for (auto odom_speed : robot_feedback.odom_speed) {
          robot_feedback_msg.odom_speed.push_back(odom_speed);
        }
        for (auto mouse_raw : robot_feedback.mouse_raw) {
          robot_feedback_msg.mouse_raw.push_back(mouse_raw);
        }
        for (auto voltage : robot_feedback.voltage) {
          robot_feedback_msg.voltage.push_back(voltage);
        }
        msg.feedback.push_back(robot_feedback_msg);
      }
      publisher->publish(msg);
    });
  }

  rclcpp::TimerBase::SharedPtr timer;

  std::vector<std::shared_ptr<MulticastReceiver>> receivers;

  rclcpp::Publisher<crane_msgs::msg::RobotFeedbackArray>::SharedPtr publisher;

  crane::ConsaiVisualizerWrapper consai_visualizer_wrapper;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<RobotReceiverNode>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
