// Copyright (c) 2024 ibis-ssl
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
  // config.ip = "224.5.20." + std::to_string(id + 100);
  config.ip = "224.5.23.2";
  config.port = 50100 + id;
  return config;
}

struct RobotFeedback
{
  uint8_t counter;

  uint8_t kick_state;

  uint8_t temperature[7];
  uint16_t error_id;
  uint16_t error_info;
  float error_value;
  float motor_current[4];
  uint8_t ball_detection[4];

  bool ball_sensor;

  float_t yaw_angle, diff_angle;
  float_t odom[2], odom_speed[2], mouse_odom[2], mouse_vel[2], voltage[2];

  uint8_t check_ver;

  std::vector<float> values;
};

union FloatUnion {
  float f;
  char b[4];
};

union Uint16Union {
  uint16_t u16;
  char b[2];
};

class MulticastReceiver
{
public:
  MulticastReceiver(const std::string & host, const int port)
  : robot_id(port - 50100), socket(io_service, boost::asio::ip::udp::v4()), buffer(2048)
  {
    boost::asio::ip::address addr = boost::asio::ip::address::from_string(host);
    if (!addr.is_multicast()) {
      throw std::runtime_error("expected multicast address");
    }

    socket.set_option(boost::asio::socket_base::reuse_address(true));
    socket.set_option(boost::asio::ip::multicast::join_group(addr.to_v4()));
    socket.bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));
    socket.non_blocking(true);
  }

  bool receive()
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

  void updateFeedback()
  {
    FloatUnion float_union;
    Uint16Union uint16_union;
    RobotFeedback feedback;
    // 最新のデータでリセット
    feedback = robot_feedback;

    // 0,1byte目は識別子みたいな感じ
    auto header = buffer[2];

    feedback.counter = buffer[3];
    {
      float_union.b[0] = buffer[4];
      float_union.b[1] = buffer[5];
      float_union.b[2] = buffer[6];
      float_union.b[3] = buffer[7];
      feedback.yaw_angle = float_union.f;
    }
    {
      float_union.b[0] = buffer[8];
      float_union.b[1] = buffer[9];
      float_union.b[2] = buffer[10];
      float_union.b[3] = buffer[11];
      feedback.voltage[0] = float_union.f;
    }

    feedback.ball_detection[0] = buffer[12];
    feedback.ball_detection[1] = buffer[13];
    feedback.ball_detection[2] = buffer[14];
    feedback.kick_state = buffer[15] * 10;

    {
      uint16_union.b[0] = buffer[16];
      uint16_union.b[1] = buffer[17];
      feedback.error_id = uint16_union.u16;
    }
    {
      uint16_union.b[0] = buffer[18];
      uint16_union.b[1] = buffer[19];
      feedback.error_info = uint16_union.u16;
    }
    {
      float_union.b[0] = buffer[20];
      float_union.b[1] = buffer[21];
      float_union.b[2] = buffer[22];
      float_union.b[3] = buffer[23];
      feedback.error_value = float_union.f;
    }

    feedback.motor_current[0] = buffer[24] / 10.;
    feedback.motor_current[1] = buffer[25] / 10.;
    feedback.motor_current[2] = buffer[26] / 10.;
    feedback.motor_current[3] = buffer[27] / 10.;

    feedback.ball_detection[3] = buffer[28];

    feedback.temperature[0] = buffer[29];
    feedback.temperature[1] = buffer[30];
    feedback.temperature[2] = buffer[31];
    feedback.temperature[3] = buffer[32];
    feedback.temperature[4] = buffer[33];
    feedback.temperature[5] = buffer[34];
    feedback.temperature[6] = buffer[35];

    {
      float_union.b[0] = buffer[36];
      float_union.b[1] = buffer[37];
      float_union.b[2] = buffer[38];
      float_union.b[3] = buffer[39];
      feedback.diff_angle = float_union.f;
    }

    {
      float_union.b[0] = buffer[40];
      float_union.b[1] = buffer[41];
      float_union.b[2] = buffer[42];
      float_union.b[3] = buffer[43];
      feedback.voltage[1] = float_union.f;
    }
    {
      float_union.b[0] = buffer[44];
      float_union.b[1] = buffer[45];
      float_union.b[2] = buffer[46];
      float_union.b[3] = buffer[47];
      feedback.odom[0] = float_union.f;
    }
    {
      float_union.b[0] = buffer[48];
      float_union.b[1] = buffer[49];
      float_union.b[2] = buffer[50];
      float_union.b[3] = buffer[51];
      feedback.odom[1] = float_union.f;
    }
    {
      float_union.b[0] = buffer[52];
      float_union.b[1] = buffer[53];
      float_union.b[2] = buffer[54];
      float_union.b[3] = buffer[55];
      feedback.odom_speed[0] = float_union.f;
    }
    {
      float_union.b[0] = buffer[56];
      float_union.b[1] = buffer[57];
      float_union.b[2] = buffer[58];
      float_union.b[3] = buffer[59];
      feedback.odom_speed[1] = float_union.f;
    }

    feedback.check_ver = buffer[60];

    {
      float_union.b[0] = buffer[64];
      float_union.b[1] = buffer[65];
      float_union.b[2] = buffer[66];
      float_union.b[3] = buffer[67];
      feedback.mouse_odom[0] = float_union.f;
    }
    {
      float_union.b[0] = buffer[68];
      float_union.b[1] = buffer[69];
      float_union.b[2] = buffer[70];
      float_union.b[3] = buffer[71];
      feedback.mouse_odom[1] = float_union.f;
    }
    {
      float_union.b[0] = buffer[72];
      float_union.b[1] = buffer[73];
      float_union.b[2] = buffer[74];
      float_union.b[3] = buffer[75];
      feedback.mouse_vel[0] = float_union.f;
    }
    {
      float_union.b[0] = buffer[76];
      float_union.b[1] = buffer[77];
      float_union.b[2] = buffer[78];
      float_union.b[3] = buffer[79];
      feedback.mouse_vel[1] = float_union.f;
    }

    for (int i = 80; i < 120 - 4; i += 4) {
      float_union.b[0] = buffer[i];
      float_union.b[1] = buffer[i + 1];
      float_union.b[2] = buffer[i + 2];
      float_union.b[3] = buffer[i + 3];
      feedback.values.push_back(float_union.f);
    }

    robot_feedback = feedback;
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
  : rclcpp::Node("robot_receiver_node"), consai_visualizer_wrapper(*this, "robot_feedback")
  {
    publisher = create_publisher<crane_msgs::msg::RobotFeedbackArray>("/robot_feedback", 10);

    for (int i = 0; i < robot_num; i++) {
      auto config = makeConfig(i);
      receivers.push_back(std::make_shared<MulticastReceiver>(config.ip, config.port));
      std::cout << "make robot receiver for id: " << static_cast<int>(i) << ", ip: " << config.ip
                << ", port: " << config.port << std::endl;
    }

    using std::chrono::operator""ms;
    timer = rclcpp::create_timer(this, get_clock(), 10ms, [&]() {
      crane_msgs::msg::RobotFeedbackArray msg;
      for (auto & receiver : receivers) {
        if (receiver->receive()) {
          receiver->updateFeedback();
        }
        auto robot_feedback = receiver->getFeedback();
        crane_msgs::msg::RobotFeedback robot_feedback_msg;
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
