// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
      boost::asio::buffer(data_, max_length), sender_endpoint,
      std::bind(&RobotInterface::handle_receive, this, _1, _2));
  }

  void handle_receive(const boost::system::error_code & error, size_t bytes_recvd)
  {
    if (!error) {
      // receive data
      //      std::string message(data_, bytes_recvd);
    }
  }

  boost::asio::io_context io_context;
  udp::socket socket;
  udp::endpoint sender_endpoint;
  enum { max_length = 1024 };
  char data_[max_length];
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
