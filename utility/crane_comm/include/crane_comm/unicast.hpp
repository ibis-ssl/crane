// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CRANE_COMM__UNICAST_HPP_
#define CRANE_COMM__UNICAST_HPP_

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <sys/socket.h>

#include <boost/asio.hpp>
#include <exception>
#include <functional>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace crane
{
namespace asio = boost::asio;

// SO_REUSEPORTソケットオプションの定義
typedef asio::detail::socket_option::boolean<SOL_SOCKET, SO_REUSEPORT> reuse_port;

// io_context / work_guard / io_thread の管理をまとめたヘルパー構造体
// 各コンポーネントでの定型的なasioスレッド管理を共通化する
struct AsioContext
{
  asio::io_context io_context;
  asio::executor_work_guard<asio::io_context::executor_type> work_guard;
  std::thread thread;

  AsioContext() : work_guard(asio::make_work_guard(io_context)) {}

  // non-copyable, non-movable
  AsioContext(const AsioContext &) = delete;
  AsioContext & operator=(const AsioContext &) = delete;

  void start()
  {
    thread = std::thread([this]() { io_context.run(); });
  }

  ~AsioContext()
  {
    work_guard.reset();
    io_context.stop();
    if (thread.joinable()) {
      thread.join();
    }
  }
};

class AsyncUdpReceiver
{
public:
  AsyncUdpReceiver(
    asio::io_context & io_ctx, const std::string & host, const int port, size_t buffer_size = 2048)
  : socket_(io_ctx, asio::ip::udp::v4()), recv_buffer_(buffer_size)
  {
    asio::ip::address addr = asio::ip::address::from_string(host);
    boost::system::error_code ec;

    socket_.set_option(asio::socket_base::reuse_address(true), ec);
    socket_.set_option(reuse_port(true), ec);

    if (addr.is_multicast()) {
      socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), port), ec);
      if (ec) {
        RCLCPP_ERROR(
          rclcpp::get_logger("crane_comm"), "[ERROR] bind失敗: %s", ec.message().c_str());
        throw std::runtime_error("bind failed: " + ec.message());
      }

      try {
        struct ifaddrs * interfaces = nullptr;
        if (getifaddrs(&interfaces) == -1) {
          throw std::runtime_error("Error: getifaddrs failed.");
        }

        std::unordered_set<std::string> joined_interfaces;
        for (struct ifaddrs * ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
          if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) continue;

          char ip[INET_ADDRSTRLEN];
          auto * addr_in = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
          inet_ntop(AF_INET, &(addr_in->sin_addr), ip, INET_ADDRSTRLEN);

          std::string if_name(ifa->ifa_name);
          if (joined_interfaces.count(if_name) > 0) continue;

          boost::asio::ip::detail::socket_option::multicast_request<
            IPPROTO_IP, IP_ADD_MEMBERSHIP, IPPROTO_IPV6, IPV6_JOIN_GROUP>
            join_device(addr.to_v4(), asio::ip::address::from_string(ip).to_v4());

          boost::system::error_code join_ec;
          socket_.set_option(join_device, join_ec);
          if (!join_ec) {
            joined_interfaces.insert(if_name);
          }
        }
        freeifaddrs(interfaces);
      } catch (std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("crane_comm"), "%s", e.what());
      }
    } else {
      socket_.bind(asio::ip::udp::endpoint(addr, port), ec);
      if (ec) {
        RCLCPP_ERROR(
          rclcpp::get_logger("crane_comm"), "[ERROR] bind失敗: %s", ec.message().c_str());
        throw std::runtime_error("bind failed: " + ec.message());
      }
    }
  }

  void startReceive(std::function<void(const std::vector<char> &, size_t)> callback)
  {
    callback_ = std::move(callback);
    running_ = true;
    doReceive();
  }

  void stop()
  {
    running_ = false;
    boost::system::error_code ec;
    socket_.cancel(ec);
  }

private:
  void doReceive()
  {
    socket_.async_receive_from(
      asio::buffer(recv_buffer_), sender_endpoint_,
      [this](const boost::system::error_code & ec, size_t bytes_transferred) {
        if (!ec) {
          try {
            callback_(recv_buffer_, bytes_transferred);
          } catch (const std::exception & e) {
            RCLCPP_ERROR(rclcpp::get_logger("crane_comm"), "受信コールバックエラー: %s", e.what());
          }
        } else if (ec != asio::error::operation_aborted) {
          RCLCPP_ERROR(rclcpp::get_logger("crane_comm"), "UDP受信エラー: %s", ec.message().c_str());
        }
        if (running_) doReceive();
      });
  }

  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint sender_endpoint_;
  std::vector<char> recv_buffer_;
  std::function<void(const std::vector<char> &, size_t)> callback_;
  bool running_ = false;
};

class AsyncUdpReceiver
{
public:
  AsyncUdpReceiver(
    asio::io_context & io_ctx, const std::string & host, const int port, size_t buffer_size = 2048)
  : socket_(io_ctx, asio::ip::udp::v4()), recv_buffer_(buffer_size)
  {
    asio::ip::address addr = asio::ip::address::from_string(host);
    boost::system::error_code ec;

    socket_.set_option(asio::socket_base::reuse_address(true), ec);
    socket_.set_option(reuse_port(true), ec);

    if (addr.is_multicast()) {
      socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), port), ec);
      if (ec) {
        RCLCPP_ERROR(
          rclcpp::get_logger("crane_comm"), "[ERROR] bind失敗: %s", ec.message().c_str());
        throw std::runtime_error("bind failed: " + ec.message());
      }

      try {
        struct ifaddrs * interfaces = nullptr;
        if (getifaddrs(&interfaces) == -1) {
          throw std::runtime_error("Error: getifaddrs failed.");
        }

        std::unordered_set<std::string> joined_interfaces;
        for (struct ifaddrs * ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
          if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) continue;

          char ip[INET_ADDRSTRLEN];
          auto * addr_in = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
          inet_ntop(AF_INET, &(addr_in->sin_addr), ip, INET_ADDRSTRLEN);

          std::string if_name(ifa->ifa_name);
          if (joined_interfaces.count(if_name) > 0) continue;

          boost::asio::ip::detail::socket_option::multicast_request<
            IPPROTO_IP, IP_ADD_MEMBERSHIP, IPPROTO_IPV6, IPV6_JOIN_GROUP>
            join_device(addr.to_v4(), asio::ip::address::from_string(ip).to_v4());

          boost::system::error_code join_ec;
          socket_.set_option(join_device, join_ec);
          if (!join_ec) {
            joined_interfaces.insert(if_name);
          }
        }
        freeifaddrs(interfaces);
      } catch (std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("crane_comm"), "%s", e.what());
      }
    } else {
      socket_.bind(asio::ip::udp::endpoint(addr, port), ec);
      if (ec) {
        RCLCPP_ERROR(
          rclcpp::get_logger("crane_comm"), "[ERROR] bind失敗: %s", ec.message().c_str());
        throw std::runtime_error("bind failed: " + ec.message());
      }
    }
  }

  void startReceive(std::function<void(const std::vector<char> &, size_t)> callback)
  {
    callback_ = std::move(callback);
    running_ = true;
    doReceive();
  }

  void stop()
  {
    running_ = false;
    boost::system::error_code ec;
    socket_.cancel(ec);
  }

private:
  void doReceive()
  {
    socket_.async_receive_from(
      asio::buffer(recv_buffer_), sender_endpoint_,
      [this](const boost::system::error_code & ec, size_t bytes_transferred) {
        if (!ec) {
          try {
            callback_(recv_buffer_, bytes_transferred);
          } catch (const std::exception & e) {
            RCLCPP_ERROR(rclcpp::get_logger("crane_comm"), "受信コールバックエラー: %s", e.what());
          }
        } else if (ec != asio::error::operation_aborted) {
          RCLCPP_ERROR(rclcpp::get_logger("crane_comm"), "UDP受信エラー: %s", ec.message().c_str());
        }
        if (running_) doReceive();
      });
  }

  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint sender_endpoint_;
  std::vector<char> recv_buffer_;
  std::function<void(const std::vector<char> &, size_t)> callback_;
  bool running_ = false;
};

}  // namespace crane

#endif  // CRANE_COMM__UNICAST_HPP_
