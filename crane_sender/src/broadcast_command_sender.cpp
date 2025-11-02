// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_sender/broadcast_command_sender.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <cstring>
#include <iostream>
#include <string>

namespace crane
{
BroadcastCommandSender::BroadcastCommandSender()
: socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
{
  try {
    // ブロードキャスト許可フラグを設定
    socket.set_option(boost::asio::socket_base::broadcast(true));
    std::cout << "✓ SO_BROADCASTフラグ設定完了" << std::endl;

    // レゾルバでエンドポイント解決
    boost::asio::ip::udp::resolver resolver(io_service);
    std::string host = CommConfig::BROADCAST_ADDRESS;
    int port = CommConfig::DEFAULT_PORT;

    boost::asio::ip::udp::resolver::query query(host, std::to_string(port));
    auto resolver_iterator = resolver.resolve(query);
    endpoint = *resolver_iterator;

    // 詳細なネットワーク設定情報を表示
    std::cout << "【実機・ブロードキャストモード初期化完了】" << std::endl;
    std::cout << "  送信先アドレス: " << host << ":" << port << std::endl;
    std::cout << "  解決されたエンドポイント: " << endpoint.address().to_string() << ":"
              << endpoint.port() << std::endl;
    std::cout << "  ローカルソケット: " << socket.local_endpoint().address().to_string() << ":"
              << socket.local_endpoint().port() << std::endl;

    // ネットワークインターフェース情報の確認
    checkNetworkInterfaces();
  } catch (const std::exception & e) {
    std::cerr << "❌ BroadcastCommandSender初期化エラー: " << e.what() << std::endl;
    throw;
  }
}

void BroadcastCommandSender::sendBroadcastPackets(
  const std::vector<std::pair<uint8_t, RobotCommandSerializedV2>> & robot_packets,
  int check_counter)
{
  char broadcast_buf[(CommConfig::AI_CMD_V2_SIZE + 1) * CommConfig::AI_CMD_V2_ROBOT_NUM] = {};

  int active_robots = 0;
  for (int i = 0; i < CommConfig::AI_CMD_V2_ROBOT_NUM && i < robot_packets.size(); i++) {
    int offset = i * (CommConfig::AI_CMD_V2_SIZE + 1);
    broadcast_buf[offset] = i;
    memcpy(&broadcast_buf[offset + 1], robot_packets[i].second.data, CommConfig::AI_CMD_V2_SIZE);

    // 空でないパケットの数をカウント
    bool is_empty = true;
    for (int j = 0; j < CommConfig::AI_CMD_V2_SIZE; j++) {
      if (robot_packets[i].second.data[j] != 0) {
        is_empty = false;
        break;
      }
    }
    if (!is_empty) active_robots++;
  }

  // パケット送信を詳細にログ出力
  size_t total_size = sizeof(broadcast_buf);
  try {
    std::cout << "📦 ブロードキャストパケット送信開始:" << std::endl;
    std::cout << "  送信先: " << endpoint.address().to_string() << ":" << endpoint.port()
              << std::endl;
    std::cout << "  パケットサイズ: " << total_size << " bytes" << std::endl;
    std::cout << "  アクティブロボット数: " << active_robots << "/"
              << CommConfig::AI_CMD_V2_ROBOT_NUM << std::endl;

    // 実際の送信処理
    size_t sent_bytes = socket.send_to(boost::asio::buffer(broadcast_buf, total_size), endpoint);

    // 送信成功ログ
    std::cout << "✅ パケット送信成功: " << sent_bytes << " bytes 送信完了" << std::endl;
  } catch (const boost::system::system_error & e) {
    std::cerr << "❌ パケット送信エラー (boost): " << e.what() << std::endl;
    std::cerr << "  エラーコード: " << e.code() << std::endl;
    std::cerr << "  エラーメッセージ: " << e.code().message() << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "❌ パケット送信例外: " << e.what() << std::endl;
  }
}

void BroadcastCommandSender::checkNetworkInterfaces() const
{
  std::cout << "🌐 利用可能なネットワークインターフェース情報:" << std::endl;

  struct ifaddrs * ifaddrs_ptr;
  if (getifaddrs(&ifaddrs_ptr) == -1) {
    std::cerr << "❌ ネットワークインターフェース情報取得エラー" << std::endl;
    return;
  }

  for (struct ifaddrs * ifa = ifaddrs_ptr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr) continue;

    // IPv4アドレスのみ表示
    if (ifa->ifa_addr->sa_family == AF_INET) {
      struct sockaddr_in * addr_in = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
      char ip_str[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &(addr_in->sin_addr), ip_str, INET_ADDRSTRLEN);

      // ブロードキャストアドレス情報も取得
      char broadcast_str[INET_ADDRSTRLEN] = "N/A";
      if (ifa->ifa_flags & IFF_BROADCAST && ifa->ifa_broadaddr) {
        struct sockaddr_in * broadcast_in =
          reinterpret_cast<struct sockaddr_in *>(ifa->ifa_broadaddr);
        inet_ntop(AF_INET, &(broadcast_in->sin_addr), broadcast_str, INET_ADDRSTRLEN);
      }

      std::cout << "  インターフェース: " << ifa->ifa_name << " IP: " << ip_str
                << " ブロードキャスト: " << broadcast_str;

      // インターフェースの状態を表示
      if (ifa->ifa_flags & IFF_UP) std::cout << " [UP]";
      if (ifa->ifa_flags & IFF_RUNNING) std::cout << " [RUNNING]";
      if (ifa->ifa_flags & IFF_BROADCAST) std::cout << " [BROADCAST]";
      std::cout << std::endl;

      // 設定されたブロードキャストアドレスとの照合
      if (std::string(broadcast_str) == CommConfig::BROADCAST_ADDRESS) {
        std::cout << "    ✅ 設定されたブロードキャストアドレスと一致!" << std::endl;
      }
    }
  }

  freeifaddrs(ifaddrs_ptr);
}
}  // namespace crane
