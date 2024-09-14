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

#ifndef ROBOCUP_SSL_COMM__MULTICAST_HPP_
#define ROBOCUP_SSL_COMM__MULTICAST_HPP_

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <boost/asio.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace multicast
{
namespace asio = boost::asio;

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

    try {
      struct ifaddrs * interfaces = nullptr;
      struct ifaddrs * ifa = nullptr;

      // ネットワークインターフェース情報の取得
      if (getifaddrs(&interfaces) == -1) {
        throw std::runtime_error("Error: getifaddrs failed.");
      }

      // ネットワークインターフェースのリストを巡回
      for (ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) {
          continue;
        }

        if (ifa->ifa_addr->sa_family == AF_INET) {  // IPv4アドレスのみ
          char ip[INET_ADDRSTRLEN];
          inet_ntop(
            AF_INET, &(((struct sockaddr_in *)ifa->ifa_addr)->sin_addr), ip, INET_ADDRSTRLEN);
          std::cout << "マルチキャスト: " << ifa->ifa_name << ": " << ip << std::endl;
          boost::asio::ip::detail::socket_option::multicast_request<
            IPPROTO_IP, IP_ADD_MEMBERSHIP, IPPROTO_IPV6, IPV6_JOIN_GROUP>
            join_device(addr.to_v4(), asio::ip::address::from_string(ip).to_v4());
          socket.set_option(join_device);
        }
      }

      freeifaddrs(interfaces);  // メモリの解放
    } catch (std::exception & e) {
      std::cerr << e.what() << std::endl;
    }

    socket.set_option(asio::socket_base::reuse_address(true));
    socket.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), port));
    socket.non_blocking(true);
  }

  size_t receive(std::vector<char> & msg)
  {
    boost::system::error_code error;
    const size_t received = socket.receive(asio::buffer(msg), 0, error);
    if (error && error != asio::error::message_size) {
      throw boost::system::system_error(error);
      return 0;
    }
    return received;
  }

  size_t available() { return socket.available(); }

private:
  asio::io_context io_context;

  asio::ip::udp::socket socket;
};

}  // namespace multicast

#endif  // ROBOCUP_SSL_COMM__MULTICAST_HPP_
