// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <math.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <boost/asio.hpp>
#include <class_loader/visibility_control.hpp>
#include <cmath>
#include <crane_msgs/msg/robot_commands.hpp>
#include <format>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "crane_sender/robot_packet.h"
#include "crane_sender/sender_base.hpp"

namespace crane
{
// 通信設定の定数
namespace CommConfig
{
constexpr int DEFAULT_PORT = 12345;
constexpr const char * BROADCAST_ADDRESS = "192.168.20.255";
constexpr int AI_CMD_V2_SIZE = 64;
constexpr int AI_CMD_V2_ROBOT_NUM = 11;
}  // namespace CommConfig

class BroadcastCommandSender
{
public:
  explicit BroadcastCommandSender()
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

  void sendBroadcastPackets(
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

private:
  void checkNetworkInterfaces() const
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

protected:
  boost::asio::io_service io_service;
  boost::asio::ip::udp::endpoint endpoint;
  boost::asio::ip::udp::socket socket;
};

class IbisSenderNode : public SenderBase
{
private:
  int debug_id;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  std::shared_ptr<BroadcastCommandSender> broadcast_sender;

  bool use_simple_velocity;

public:
  CLASS_LOADER_PUBLIC
  explicit IbisSenderNode(const rclcpp::NodeOptions & options) : SenderBase("ibis_sender", options)
  {
    declare_parameter("debug_id", -1);
    get_parameter("debug_id", debug_id);

    declare_parameter("use_simple_velocity", false);
    get_parameter("use_simple_velocity", use_simple_velocity);

    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback("debug_id", [&](const rclcpp::Parameter & p) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          debug_id = p.as_int();
        } else {
          std::cout << "警告: debug_idは整数である必要があります" << std::endl;
        }
      });

    // ブロードキャスト送信システム初期化
    broadcast_sender = std::make_shared<BroadcastCommandSender>();
    std::cout << "ibis_sender_node 開始" << std::endl;
  }

private:
  crane_msgs::msg::RobotCommands createTestRobotCommands() const
  {
    crane_msgs::msg::RobotCommands test_commands;
    test_commands.robot_commands.clear();

    static double time_offset = 0.0;
    time_offset += 0.5;  // 固定0.5秒間隔

    const int test_robot_count = 2;  // 固定2台
    for (int i = 0; i < test_robot_count && i < CommConfig::AI_CMD_V2_ROBOT_NUM; i++) {
      crane_msgs::msg::RobotCommand cmd;

      // 基本設定
      cmd.robot_id = i;

      // 動的な位置設定（円運動パターン）
      double angle = time_offset * 0.5 + i * (2.0 * M_PI / test_robot_count);
      double radius = 1.0 + 0.5 * std::sin(time_offset * 0.3);
      cmd.current_pose.x = radius * std::cos(angle);
      cmd.current_pose.y = radius * std::sin(angle);
      cmd.current_pose.theta = angle + M_PI / 2;  // 進行方向を向く

      // 目標角度（時間に応じて回転）
      cmd.target_theta = time_offset * 0.2 + i * M_PI / 4;

      // キック・ドリブル動的設定
      cmd.kick_power = std::abs(std::sin(time_offset + i)) * 15.0;          // 0-15の範囲
      cmd.dribble_power = std::abs(std::cos(time_offset * 0.7 + i)) * 0.8;  // 0-0.8の範囲

      // チップキックは交互に
      cmd.chip_enable = ((static_cast<int>(time_offset) + i) % 4 == 0);

      // ドリブラー上げ下げ
      cmd.lift_up_dribbler_flag = ((static_cast<int>(time_offset * 2) + i) % 3 == 0);

      // 停止フラグは基本的にfalse
      cmd.stop_flag = false;

      // 制御モード（サイクルで切り替え）
      int mode_cycle = static_cast<int>(time_offset / 3) % 3;
      switch (mode_cycle) {
        case 0:  // POSITION_TARGET_MODE
          cmd.control_mode = crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE;
          {
            crane_msgs::msg::PositionTargetMode pos_mode;
            pos_mode.target_x = cmd.current_pose.x + 0.5 * std::cos(time_offset);
            pos_mode.target_y = cmd.current_pose.y + 0.5 * std::sin(time_offset);
            cmd.position_target_mode.push_back(pos_mode);
          }
          break;
        case 1:  // SIMPLE_VELOCITY_TARGET_MODE
          cmd.control_mode = crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE;
          {
            crane_msgs::msg::SimpleVelocityTargetMode vel_mode;
            vel_mode.target_vx = std::cos(time_offset + i) * 2.0;
            vel_mode.target_vy = std::sin(time_offset + i) * 2.0;
            cmd.simple_velocity_target_mode.push_back(vel_mode);
          }
          break;
        case 2:  // POLAR_VELOCITY_TARGET_MODE
          cmd.control_mode = crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE;
          {
            crane_msgs::msg::PolarVelocityTargetMode polar_mode;
            polar_mode.target_velocity_r = 1.5 + 0.5 * std::sin(time_offset);
            polar_mode.target_velocity_theta = time_offset * 0.5 + i * M_PI / 3;
            cmd.polar_velocity_target_mode.push_back(polar_mode);
          }
          break;
      }

      // プランナー設定
      cmd.local_planner_config.max_velocity = 2.0 + std::sin(time_offset) * 0.5;
      cmd.local_planner_config.max_acceleration = 3.0 + std::cos(time_offset * 0.8) * 0.5;
      cmd.local_planner_config.terminal_velocity = 0.1;
      cmd.omega_limit = 5.0;

      // タイミング情報
      cmd.elapsed_time_ms_since_last_vision = static_cast<uint32_t>((time_offset * 1000)) % 100;

      test_commands.robot_commands.push_back(cmd);
    }

    return test_commands;
  }

  RobotCommandV2 createRobotPacket(const crane_msgs::msg::RobotCommand & command, int counter)
  {
    RobotCommandV2 packet;
    packet.header = 0x00;
    packet.check_counter = counter;
    packet.vision_global_pos[0] = command.current_pose.x;
    packet.vision_global_pos[1] = command.current_pose.y;
    packet.vision_global_theta = command.current_pose.theta;
    packet.is_vision_available = [&]() -> bool {
      std::vector<uint8_t> available_ids = world_model->ours().getAvailableRobotIds();
      return std::count(available_ids.begin(), available_ids.end(), command.robot_id) == 1;
    }();
    packet.target_global_theta = command.target_theta;
    packet.kick_power = command.kick_power;
    packet.dribble_power = std::clamp(command.dribble_power, 0.f, 1.f);
    packet.enable_chip = command.chip_enable;
    packet.lift_dribbler = command.lift_up_dribbler_flag;
    packet.stop_emergency = command.stop_flag;
    packet.acceleration_limit = command.local_planner_config.max_acceleration + 1.0;
    packet.linear_velocity_limit = command.local_planner_config.max_velocity;
    packet.angular_velocity_limit = command.omega_limit;
    packet.prioritize_move = true;
    packet.prioritize_accurate_acceleration = true;
    packet.latency_time_ms = 100;

    packet.elapsed_time_ms_since_last_vision = command.elapsed_time_ms_since_last_vision;

    switch (command.control_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE: {
        packet.control_mode = POSITION_TARGET_MODE;
        if (not command.position_target_mode.empty()) {
          packet.mode_args.position.target_global_pos[0] =
            command.position_target_mode.front().target_x;
          packet.mode_args.position.target_global_pos[1] =
            command.position_target_mode.front().target_y;
          packet.mode_args.position.terminal_velocity =
            command.local_planner_config.terminal_velocity;
        } else {
          packet.mode_args.position.target_global_pos[0] = 0.0;
          packet.mode_args.position.target_global_pos[1] = 0.0;
          packet.mode_args.position.terminal_velocity = 0.0;
          std::cout << "警告: position_target_modeが空です" << std::endl;
        }
      } break;
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE: {
        if (not command.simple_velocity_target_mode.empty()) {
          const auto & simple_velocity = command.simple_velocity_target_mode.front();
          if (use_simple_velocity) {
            // simple -> simple
            packet.control_mode = SIMPLE_VELOCITY_TARGET_MODE;
            packet.mode_args.simple_velocity.target_global_vel[0] = simple_velocity.target_vx;
            packet.mode_args.simple_velocity.target_global_vel[1] = simple_velocity.target_vy;
          } else {
            // simple -> polar
            packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
            packet.mode_args.polar_velocity.target_global_velocity_r =
              std::hypot(simple_velocity.target_vx, simple_velocity.target_vy);
            packet.mode_args.polar_velocity.target_global_velocity_theta =
              std::atan2(simple_velocity.target_vy, simple_velocity.target_vx);
          }
        } else {
          std::cout << "警告: simple_velocity_target_modeが空です" << std::endl;
        }
      } break;
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE: {
        packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
        if (not command.polar_velocity_target_mode.empty()) {
          const auto & polar_velocity = command.polar_velocity_target_mode.front();
          if (use_simple_velocity) {
            // polar -> simple
            packet.control_mode = SIMPLE_VELOCITY_TARGET_MODE;
            packet.mode_args.simple_velocity.target_global_vel[0] =
              polar_velocity.target_velocity_r * std::cos(polar_velocity.target_velocity_theta);
            packet.mode_args.simple_velocity.target_global_vel[1] =
              polar_velocity.target_velocity_r * std::sin(polar_velocity.target_velocity_theta);
          } else {
            // polar -> polar
            packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
            packet.mode_args.polar_velocity.target_global_velocity_r =
              polar_velocity.target_velocity_r;
            packet.mode_args.polar_velocity.target_global_velocity_theta =
              polar_velocity.target_velocity_theta;
          }
        } else {
          std::cout << "警告: polar_velocity_target_modeが空です" << std::endl;
        }
      } break;
      case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE: {
        packet.control_mode = LOCAL_CAMERA_MODE;
        if (not command.local_camera_mode.empty()) {
          packet.mode_args.local_camera.target_global_vel[0] =
            command.local_camera_mode.front().target_global_vx;
          packet.mode_args.local_camera.target_global_vel[1] =
            command.local_camera_mode.front().target_global_vy;
        } else {
          std::cout << "警告: local_camera_modeが空です" << std::endl;
        }
      } break;
      default:
        std::cout << "エラー: 無効な制御モードです" << std::endl;
        break;
    }
    return packet;
  }

public:
  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    static int counter = 0;
    static int call_count = 0;
    call_count++;

    if (++counter > 200) {
      counter = 0;
    }

    // メソッド呼び出し確認ログ
    std::cout << "🚀 sendCommands メソッド呼び出し #" << call_count << " (カウンタ=" << counter
              << ")" << std::endl;
    std::cout << "  受信したロボットコマンド数: " << msg.robot_commands.size() << std::endl;

    // ブロードキャストモード：全ロボットのパケットを作成して一括送信
    std::vector<std::pair<uint8_t, RobotCommandSerializedV2>> robot_packets;

    // 全11スロット分の配列を初期化（ロボットIDでインデックス指定）
    std::array<std::optional<RobotCommandSerializedV2>, CommConfig::AI_CMD_V2_ROBOT_NUM>
      packet_slots;

    int processed_commands = 0;
    for (auto command : msg.robot_commands) {
      if (command.robot_id < CommConfig::AI_CMD_V2_ROBOT_NUM) {
        RobotCommandV2 packet = createRobotPacket(command, counter);
        RobotCommandSerializedV2 serialized_packet;
        RobotCommandSerializedV2_serialize(&serialized_packet, &packet);
        packet_slots[command.robot_id] = serialized_packet;
        processed_commands++;
      }
    }

    std::cout << "  処理されたコマンド数: " << processed_commands << "/"
              << msg.robot_commands.size() << std::endl;

    // 全スロットを順番にパケットリストに追加（空のスロットは空パケット）
    int filled_slots = 0;
    for (int i = 0; i < CommConfig::AI_CMD_V2_ROBOT_NUM; i++) {
      if (packet_slots[i]) {
        robot_packets.push_back(std::make_pair(i, packet_slots[i].value()));
        filled_slots++;
      } else {
        // 空パケットを作成
        RobotCommandSerializedV2 empty_packet = {};
        robot_packets.push_back(std::make_pair(i, empty_packet));
      }
    }

    std::cout << "  パケットスロット使用状況: " << filled_slots << "/"
              << CommConfig::AI_CMD_V2_ROBOT_NUM << " スロット使用中" << std::endl;
    std::cout << "  ブロードキャスト送信準備完了 → パケット送信開始" << std::endl;

    broadcast_sender->sendBroadcastPackets(robot_packets, counter);

    // 定期的な詳細ログ（100回に1回）
    if (call_count % 100 == 0) {
      std::cout << "📈 統計情報 (100回毎): 総呼び出し=" << call_count
                << " 平均コマンド数=" << (msg.robot_commands.size()) << std::endl;
    }
  }
};
}  // namespace crane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<crane::IbisSenderNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
