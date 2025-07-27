// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <math.h>
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
#include <crane_msgs/msg/robot_commands.hpp>
#include <format>
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
// 送信方式の定義
enum class SendMode { SIMULATION, REAL };
enum class SendType { UNICAST, BROADCAST };

// 通信設定の定数
namespace CommConfig
{
constexpr int DEFAULT_PORT = 12345;
constexpr int SIMULATION_BASE_PORT = 50100;
constexpr const char * BROADCAST_ADDRESS = "224.5.20.255";
constexpr const char * LOCALHOST = "localhost";
constexpr int AI_CMD_V2_SIZE = 64;
constexpr int AI_CMD_V2_ROBOT_NUM = 11;
}  // namespace CommConfig

class CommandSenderBase
{
public:
  CommandSenderBase(SendType type, SendMode mode) : send_type_(type), send_mode_(mode) {}
  virtual ~CommandSenderBase() = default;

protected:
  std::string getConnectionInfo(const std::string & host, int port) const
  {
    std::string type_str = (send_type_ == SendType::UNICAST) ? "ユニキャスト" : "ブロードキャスト";
    std::string mode_str = (send_mode_ == SendMode::SIMULATION) ? "シミュレーション" : "実機";
    return std::format("[{}・{}] 送信先: {}:{}", type_str, mode_str, host, port);
  }

  SendType send_type_;
  SendMode send_mode_;
};

class UnicastCommandSender : public CommandSenderBase
{
public:
  explicit UnicastCommandSender(uint8_t robot_id, bool sim_mode)
  : CommandSenderBase(SendType::UNICAST, sim_mode ? SendMode::SIMULATION : SendMode::REAL),
    socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
  {
    boost::asio::ip::udp::resolver resolver(io_service);
    endpoint = [&]() -> boost::asio::ip::udp::endpoint {
      if (sim_mode) {
        std::string host = CommConfig::LOCALHOST;
        int port = CommConfig::SIMULATION_BASE_PORT + robot_id;
        boost::asio::ip::udp::resolver::query query(host, std::to_string(port));
        std::cout << getConnectionInfo(host, port) << " ロボット" << static_cast<int>(robot_id)
                  << std::endl;
        return *resolver.resolve(query);
      } else {
        std::string host = std::format("192.168.20.{}", 100 + robot_id);
        int port = CommConfig::DEFAULT_PORT;
        boost::asio::ip::udp::resolver::query query(host, std::to_string(port));
        std::cout << getConnectionInfo(host, port) << " ロボット" << static_cast<int>(robot_id)
                  << std::endl;
        return *resolver.resolve(query);
      }
    }();
  }

  RobotCommandSerializedV2 send(RobotCommandV2 packet, int check_counter)
  {
    packet.check_counter = check_counter;
    RobotCommandSerializedV2 serialized_packet;
    RobotCommandSerializedV2_serialize(&serialized_packet, &packet);

    uint8_t send_packet[64] = {};
    for (int i = 0; i < 64; ++i) {
      send_packet[i] = serialized_packet.data[i];
    }

    socket.send_to(boost::asio::buffer(send_packet), endpoint);

    return serialized_packet;
  }

protected:
  boost::asio::io_service io_service;

  boost::asio::ip::udp::endpoint endpoint;

  boost::asio::ip::udp::socket socket;

  int check = 0;
};

class BroadcastCommandSender : public CommandSenderBase
{
public:
  explicit BroadcastCommandSender(bool sim_mode)
  : CommandSenderBase(SendType::BROADCAST, sim_mode ? SendMode::SIMULATION : SendMode::REAL),
    socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
  {
    boost::asio::ip::udp::resolver resolver(io_service);
    std::string host = sim_mode ? CommConfig::LOCALHOST : CommConfig::BROADCAST_ADDRESS;
    int port = CommConfig::DEFAULT_PORT;

    boost::asio::ip::udp::resolver::query query(host, std::to_string(port));
    endpoint = *resolver.resolve(query);
    std::cout << getConnectionInfo(host, port) << std::endl;
  }

  void sendBroadcastPackets(
    const std::vector<std::pair<uint8_t, RobotCommandSerializedV2>> & robot_packets,
    int check_counter)
  {
    char broadcast_buf[(CommConfig::AI_CMD_V2_SIZE + 1) * CommConfig::AI_CMD_V2_ROBOT_NUM] = {};

    int active_robots = 0;
    for (int i = 0; i < CommConfig::AI_CMD_V2_ROBOT_NUM && i < robot_packets.size(); i++) {
      int offset = i * (CommConfig::AI_CMD_V2_SIZE + 1);
      broadcast_buf[offset] = AF_INET;  // Orion_CM4が期待する識別子（AF_INET = 2）
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

    socket.send_to(boost::asio::buffer(broadcast_buf, sizeof(broadcast_buf)), endpoint);

    // デバッグ出力（Orion_CM4の出力形式に合わせて）
    static int last_check_counter = -1;
    if (check_counter != last_check_counter) {
      std::string type_str =
        (send_type_ == SendType::BROADCAST) ? "ブロードキャスト" : "ユニキャスト";
      std::cout << type_str << "パケット送信完了: アクティブロボット数=" << active_robots
                << " チェックカウンタ=" << check_counter << std::endl;
      last_check_counter = check_counter;
    }
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

  std::array<std::shared_ptr<UnicastCommandSender>, 20> senders;

  std::shared_ptr<BroadcastCommandSender> broadcast_sender;

  bool sim_mode;

  bool use_simple_velocity;

  bool use_broadcast_mode;
  SendMode current_send_mode;

public:
  CLASS_LOADER_PUBLIC
  explicit IbisSenderNode(const rclcpp::NodeOptions & options) : SenderBase("ibis_sender", options)
  {
    declare_parameter("debug_id", -1);
    get_parameter("debug_id", debug_id);

    declare_parameter("sim_mode", true);
    get_parameter("sim_mode", sim_mode);

    declare_parameter("use_simple_velocity", false);
    get_parameter("use_simple_velocity", use_simple_velocity);

    declare_parameter("use_broadcast_mode", false);
    get_parameter("use_broadcast_mode", use_broadcast_mode);

    current_send_mode = sim_mode ? SendMode::SIMULATION : SendMode::REAL;
    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback("debug_id", [&](const rclcpp::Parameter & p) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          debug_id = p.as_int();
        } else {
          std::cout << "警告: debug_idは整数である必要があります" << std::endl;
        }
      });

    for (std::size_t i = 0; i < senders.size(); i++) {
      senders[i] = std::make_shared<UnicastCommandSender>(i, sim_mode);
    }

    // 送信システム初期化
    std::string mode_str =
      (current_send_mode == SendMode::SIMULATION) ? "シミュレーション" : "実機";
    if (use_broadcast_mode) {
      broadcast_sender = std::make_shared<BroadcastCommandSender>(sim_mode);
      std::cout << "【" << mode_str << "・ブロードキャストモード】 ibis_sender_node 開始"
                << std::endl;
    } else {
      std::cout << "【" << mode_str << "・ユニキャストモード】 ibis_sender_node 開始" << std::endl;
    }
  }

private:
  RobotCommandV2 createRobotPacket(const crane_msgs::msg::RobotCommand & command, int /* counter */)
  {
    RobotCommandV2 packet;
    packet.header = 0x00;
    packet.check_counter = 0;
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
    if (++counter > 200) {
      counter = 0;
    }

    if (use_broadcast_mode && broadcast_sender) {
      // ブロードキャストモード：全ロボットのパケットを作成して一括送信
      std::vector<std::pair<uint8_t, RobotCommandSerializedV2>> robot_packets;

      // 全11スロット分の配列を初期化（ロボットIDでインデックス指定）
      std::array<std::optional<RobotCommandSerializedV2>, CommConfig::AI_CMD_V2_ROBOT_NUM>
        packet_slots;

      for (auto command : msg.robot_commands) {
        if (command.robot_id < CommConfig::AI_CMD_V2_ROBOT_NUM) {
          RobotCommandV2 packet = createRobotPacket(command, counter);
          RobotCommandSerializedV2 serialized_packet;
          RobotCommandSerializedV2_serialize(&serialized_packet, &packet);
          packet_slots[command.robot_id] = serialized_packet;
        }
      }

      // 全スロットを順番にパケットリストに追加（空のスロットは空パケット）
      for (int i = 0; i < CommConfig::AI_CMD_V2_ROBOT_NUM; i++) {
        if (packet_slots[i]) {
          robot_packets.push_back(std::make_pair(i, packet_slots[i].value()));
        } else {
          // 空パケットを作成
          RobotCommandSerializedV2 empty_packet = {};
          robot_packets.push_back(std::make_pair(i, empty_packet));
        }
      }

      broadcast_sender->sendBroadcastPackets(robot_packets, counter);
    } else {
      // 個別送信モード：従来通りの処理
      for (auto command : msg.robot_commands) {
        RobotCommandV2 packet = createRobotPacket(command, counter);
        senders[command.robot_id]->send(packet, counter);
      }
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
