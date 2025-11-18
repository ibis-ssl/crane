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
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <format>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "crane_sender/broadcast_command_sender.hpp"
#include "crane_sender/robot_packet.h"
#include "crane_sender/sender_base.hpp"

namespace crane
{

class IbisSenderNode : public SenderBase
{
private:
  int debug_id;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  std::shared_ptr<BroadcastCommandSender> broadcast_sender;

  // ロボット送信用の加速度パラメータ
  double robot_acceleration_acceleration;
  double robot_acceleration_deceleration_high;
  double robot_acceleration_deceleration_low;
  double robot_acceleration_velocity_threshold;

public:
  CLASS_LOADER_PUBLIC
  explicit IbisSenderNode(const rclcpp::NodeOptions & options) : SenderBase("ibis_sender", options)
  {
    declare_parameter("debug_id", -1);
    get_parameter("debug_id", debug_id);

    // ロボット送信用の加速度パラメータを読み込み
    declare_parameter("robot_acceleration.acceleration", 2.5);
    get_parameter("robot_acceleration.acceleration", robot_acceleration_acceleration);

    declare_parameter("robot_acceleration.deceleration_high", 3.0);
    get_parameter("robot_acceleration.deceleration_high", robot_acceleration_deceleration_high);

    declare_parameter("robot_acceleration.deceleration_low", 2.0);
    get_parameter("robot_acceleration.deceleration_low", robot_acceleration_deceleration_low);

    declare_parameter("robot_acceleration.velocity_threshold", 1.5);
    get_parameter("robot_acceleration.velocity_threshold", robot_acceleration_velocity_threshold);

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
    packet.stop_emergency = command.stop_flag;

    // 現在の速度から加速度を選択
    double current_speed = std::hypot(command.current_velocity.x, command.current_velocity.y);
    double target_speed = command.local_planner_config.final_planned_max_velocity.value;

    double selected_acceleration;
    if (current_speed < target_speed) {
      // 加速時
      selected_acceleration = robot_acceleration_acceleration;
    } else {
      // 減速時：速度に応じて選択
      if (current_speed >= robot_acceleration_velocity_threshold) {
        // 高速域
        selected_acceleration = robot_acceleration_deceleration_high;
      } else {
        // 低速域
        selected_acceleration = robot_acceleration_deceleration_low;
      }
    }

    packet.acceleration_limit = selected_acceleration;
    packet.linear_velocity_limit = command.local_planner_config.final_planned_max_velocity.value;
    packet.angular_velocity_limit = command.omega_limit;
    packet.latency_time_ms = static_cast<uint8_t>(command.latency_ms);
    packet.elapsed_time_ms_since_last_vision = command.elapsed_time_ms_since_last_vision;

    switch (command.control_mode) {
      case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE:
        break;
      case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE: {
        if (not command.simple_velocity_target_mode.empty()) {
          const auto & simple_velocity = command.simple_velocity_target_mode.front();
          // simple -> polar
          packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
          packet.mode_args.polar_velocity.target_global_velocity_r =
            std::hypot(simple_velocity.target_vx, simple_velocity.target_vy);
          packet.mode_args.polar_velocity.target_global_velocity_theta =
            std::atan2(simple_velocity.target_vy, simple_velocity.target_vx);
        } else {
          std::cout << "警告: simple_velocity_target_modeが空です" << std::endl;
        }
      } break;
      case crane_msgs::msg::RobotCommand::POLAR_VELOCITY_TARGET_MODE: {
        packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
        if (not command.polar_velocity_target_mode.empty()) {
          const auto & polar_velocity = command.polar_velocity_target_mode.front();
          // polar -> polar
          packet.control_mode = POLAR_VELOCITY_TARGET_MODE;
          packet.mode_args.polar_velocity.target_global_velocity_r =
            polar_velocity.target_velocity_r;
          packet.mode_args.polar_velocity.target_global_velocity_theta =
            polar_velocity.target_velocity_theta;
        } else {
          std::cout << "警告: polar_velocity_target_modeが空です" << std::endl;
        }
      } break;
      case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE:
        break;
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
