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

#ifndef CRANE_COMM__MULTICAST_HPP_
#define CRANE_COMM__MULTICAST_HPP_

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <sys/socket.h>

#include <boost/asio.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace crane
{
namespace asio = boost::asio;

// SO_REUSEPORTソケットオプションの定義
typedef asio::detail::socket_option::boolean<SOL_SOCKET, SO_REUSEPORT> reuse_port;

class MulticastReceiver
{
public:
  MulticastReceiver(const std::string & host, const int port)
  : socket(io_context, asio::ip::udp::v4())
  {
    asio::ip::address addr = asio::ip::address::from_string(host);
    if (!addr.is_multicast()) {
      throw std::runtime_error("expected multicast address");
    }

    // ソケットオプションを先に設定
    boost::system::error_code ec;

    socket.set_option(asio::socket_base::reuse_address(true), ec);

    socket.set_option(reuse_port(true), ec);

    socket.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), port), ec);
    if (ec) {
      std::cerr << "[ERROR] bind失敗: " << ec.message() << std::endl;
      throw std::runtime_error("bind failed: " + ec.message());
    }

    socket.non_blocking(true);

    // その後マルチキャストグループに参加
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

        if (ifa->ifa_addr->sa_family == AF_INET) {  // IPv4アドレスのみ
          char ip[INET_ADDRSTRLEN];
          struct sockaddr_in * addr_in =
            (struct sockaddr_in *)ifa->ifa_addr;  // キャスト後に変数に格納
          inet_ntop(AF_INET, &(addr_in->sin_addr), ip, INET_ADDRSTRLEN);

          interface_count++;
          // 同じインターフェースで既に参加済みの場合はスキップ
          std::string if_name(ifa->ifa_name);
          std::cout << "マルチキャスト: " << ifa->ifa_name << ": " << ip << std::endl;
          if (joined_interfaces.count(if_name) > 0) {
            skip_count++;
            continue;
          }

          boost::asio::ip::detail::socket_option::multicast_request<
            IPPROTO_IP, IP_ADD_MEMBERSHIP, IPPROTO_IPV6, IPV6_JOIN_GROUP>
            join_device(addr.to_v4(), asio::ip::address::from_string(ip).to_v4());

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
      std::cerr << e.what() << std::endl;
    }
  }

  auto receive(std::vector<char> & msg) -> size_t
  {
    boost::system::error_code error;
    const size_t received = socket.receive(asio::buffer(msg), 0, error);
    if (error && error != asio::error::message_size) {
      throw boost::system::system_error(error);
      return 0;
    }
    return received;
  }

  auto available() const -> size_t { return socket.available(); }

private:
  asio::io_context io_context;

  asio::ip::udp::socket socket;
};

}  // namespace crane

#endif  // CRANE_COMM__MULTICAST_HPP_
