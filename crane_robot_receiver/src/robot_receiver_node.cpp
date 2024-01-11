// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <atomic>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <crane_msgs/msg/robot_feed_back.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using boost::asio::ip::udp;
using namespace std::placeholders;

struct RobotInterfaceConfig
{
  std::string ip;
  int port;
};

auto makeConfig(uint8_t id)
{
  RobotInterfaceConfig config;
  config.ip = "224.5.20." + std::to_string(id);
  config.port = 50000 + id;
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

struct RobotInterface
{
  RobotInterface(const RobotInterfaceConfig config)
  : socket(io_context, udp::endpoint(udp::v4(), config.port)), config(config)
  {
  }

  void open()
  {
    socket.open(endpoint.protocol());
    socket.set_option(udp::socket::reuse_address(true));
    socket.bind(endpoint);

    auto address = boost::asio::ip::make_address(config.ip);
    socket.set_option(boost::asio::ip::multicast::join_group(address));

    socket.async_receive_from(
      boost::asio::buffer(buffer, max_length), sender_endpoint,
      std::bind(&RobotInterface::handle_receive, this, _1, _2));
  }

  void handle_receive(const boost::system::error_code & error, size_t bytes)
  {
    if (error) {
      return;
    }

    FloatUnion float_union;
    RobotInfoUnion robot_info_union;
    robot_info_union.data = robot_feedback.load();

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
    robot_feedback.store(robot_info_union.data);
  }

  // この変数は非同期で読み書きするのでatomicにする
  std::atomic<RobotFeedback> robot_feedback;

  boost::asio::io_context io_context;

  udp::socket socket;

  udp::endpoint sender_endpoint;

  enum { max_length = 1024 };

  char buffer[max_length];

  udp::endpoint endpoint;

  const RobotInterfaceConfig config;
};

class RobotReceiverNode : public rclcpp::Node
{
public:
  RobotReceiverNode()
  : rclcpp::Node("robot_receiver_node"), socket_(io_context_), endpoint_(udp::v4(), 30001)
  {
    publisher = create_publisher<crane_msgs::msg::RobotFeedBack>("/robot_feedback", 10);

    socket_.open(endpoint_.protocol());
    socket_.set_option(udp::socket::reuse_address(true));
    socket_.bind(endpoint_);

    auto address = boost::asio::ip::make_address("224.5.20.102");
    socket_.set_option(boost::asio::ip::multicast::join_group(address));

    startReceiving();

    io_thread = std::make_unique<std::thread>([this]() { io_context_.run(); });
  }

  ~RobotReceiverNode()
  {
    io_context_.stop();
    if (io_thread && io_thread->joinable()) {
      io_thread->join();
    }
  }

  void startReceiving()
  {
    socket_.async_receive_from(
      boost::asio::buffer(data_, max_length), sender_endpoint_,
      std::bind(&RobotReceiverNode::callbackUDPPacket, this, _1, _2));
  }

  void callbackUDPPacket(const boost::system::error_code & error, size_t bytes_recvd)
  {
    if (!error) {
      std::string message(data_, bytes_recvd);

      // ROS 2 にメッセージを publish
      //      auto msg = std_msgs::msg::String();
      //      msg.data = message;
      //      publisher_->publish(msg);

      startReceiving();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  boost::asio::ip::address multicast_address;
  boost::asio::io_context io_context_;
  udp::socket socket_;
  udp::endpoint sender_endpoint_;
  enum { max_length = 1024 };
  char data_[max_length];
  udp::endpoint endpoint_;
  std::unique_ptr<std::thread> io_thread;
  rclcpp::Publisher<crane_msgs::msg::RobotFeedBack>::SharedPtr publisher;
};
int main()
{
  std::cout << "hello, this is robot_receiver_node" << std::endl;
  return 0;
}
