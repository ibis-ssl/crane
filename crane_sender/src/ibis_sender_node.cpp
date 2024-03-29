// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <arpa/inet.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
// #include <netinet/in.h>
#include <netinet/udp.h>

#include <class_loader/visibility_control.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "crane_sender/robot_packet.hpp"
#include "crane_sender/sender_base.hpp"

int check;

namespace crane
{
class IbisSenderNode : public SenderBase
{
private:
  int debug_id;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  WorldModelWrapper::SharedPtr world_model;

  int sock;

  std::string interface;

  bool sim_mode;

public:
  CLASS_LOADER_PUBLIC
  explicit IbisSenderNode(const rclcpp::NodeOptions & options) : SenderBase("real_sender", options)
  {
    declare_parameter("debug_id", -1);
    get_parameter("debug_id", debug_id);
    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback("debug_id", [&](const rclcpp::Parameter & p) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          debug_id = p.as_int();
        } else {
          std::cout << "debug_id is not integer" << std::endl;
        }
      });

    declare_parameter("interface", "enp4s0");
    get_parameter("interface", interface);

    declare_parameter("sim", false);
    get_parameter("sim", sim_mode);
    if (sim_mode) {
      interface = "lo";
    }

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    // cspell: ignore BINDTODEVICE
    setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, interface.c_str(), 4);

    world_model = std::make_shared<WorldModelWrapper>(*this);

    std::cout << "start" << std::endl;
  }

  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    if (not world_model->hasUpdated()) {
      return;
    }
    uint8_t send_packet[32] = {};

    auto to_two_byte = [](float val, float range) -> std::pair<uint8_t, uint8_t> {
      uint16_t two_byte = static_cast<int>(32767 * static_cast<float>(val / range) + 32767);
      uint8_t byte_low, byte_high;
      byte_low = two_byte & 0x00FF;
      byte_high = (two_byte & 0xFF00) >> 8;
      return std::make_pair(byte_low, byte_high);
    };

    auto normalize_angle = [](float angle_rad) -> float {
      if (fabs(angle_rad) > M_PI) {
        while (angle_rad > M_PI) {
          angle_rad -= 2.0f * M_PI;
        }
        while (angle_rad < -M_PI) {
          angle_rad += 2.0f * M_PI;
        }
      }
      return angle_rad;
    };

    for (auto command : msg.robot_commands) {
      //
      if (not msg.on_positive_half) {
        command.target_velocity.x *= -1;
        command.target_velocity.y *= -1;
        command.target_velocity.theta *= -1;
        if (not command.target_theta.empty()) {
          command.target_theta.front() *= -1;
        }

        if (not command.target_x.empty()) {
          command.target_x.front() *= -1;
        }

        if (not command.target_y.empty()) {
          command.target_y.front() *= -1;
        }

        command.current_pose.x *= -1.;
        command.current_pose.y *= -1.;
        command.current_pose.theta *= -1.;
        command.current_ball_x *= -1.;
        command.current_ball_y *= -1.;
      }
      // vel_surge
      //  -7 ~ 7 -> 0 ~ 32767 ~ 65534
      // 取り敢えず横偏差をなくすためにy方向だけゲインを高めてみる
      if (std::abs(command.target_velocity.y) < 0.3) {
        //        command.target_velocity.y *= 8.f;
      }

      RobotCommand packet;
      packet.VEL_LOCAL_SURGE = command.target_velocity.x;
      packet.VEL_LOCAL_SWAY = command.target_velocity.y;

      packet.DRIBBLE_POWER = std::clamp(command.dribble_power, 0.f, 1.f);
      packet.KICK_POWER = std::clamp(command.kick_power, 0.f, 1.f);
      packet.CHIP_ENABLE = command.chip_enable;

      // -pi ~ pi -> 0 ~ 32767 ~ 65534
      packet.TARGET_GLOBAL_THETA = [&]() -> float {
        if (not command.target_theta.empty()) {
          return normalize_angle(command.target_theta.front());
        } else {
          return 0.f;
        }
      }();

      // Vision角度
      // -pi ~ pi -> 0 ~ 32767 ~ 65534
      packet.VISION_GLOBAL_X = command.current_pose.x;
      packet.VISION_GLOBAL_Y = command.current_pose.y;
      packet.VISION_GLOBAL_THETA = command.current_pose.theta;

      packet.BALL_GLOBAL_X = command.current_ball_x;
      packet.BALL_GLOBAL_Y = command.current_ball_y;

      // 目標座標
      packet.TARGET_GLOBAL_X = [&]() -> float {
        if (not command.target_x.empty()) {
          return command.target_x.front();
        } else {
          return 0.f;
        }
      }();

      packet.TARGET_GLOBAL_Y = [&]() -> float {
        if (not command.target_y.empty()) {
          return command.target_y.front();
        } else {
          return 0.f;
        }
      }();
      packet.LOCAL_FEEDBACK_ENABLE = [&]() -> bool { return false; }();

      std::vector<uint8_t> available_ids = world_model->ours.getAvailableRobotIds();
      packet.IS_ID_VISIBLE =
        std::count(available_ids.begin(), available_ids.end(), command.robot_id) == 1;
      packet.STOP_FLAG = command.stop_flag;
      packet.IS_DRIBBLER_UP = command.lift_up_dribbler_flag;
      // キーパーEN
      // 0 or 1

      packet.LOCAL_KEEPER_MODE_ENABLE = command.local_goalie_enable;

      packet.CHECK = check;

      RobotCommandSerialized serialized_packet(packet);

      for (int i = 0; i < static_cast<int>(RobotCommandSerialized::Address::SIZE); ++i) {
        send_packet[i] = serialized_packet.data[i];
      }

      if (command.robot_id == debug_id) {
        std::stringstream ss;
        for (int i = 0; i < 25; ++i) {
          ss << std::hex << static_cast<int>(send_packet[i]) << " ";
        }
        std::cout << ss.str() << std::endl;
      }
      check++;
      if (check > 200) {
        check = 0;
      }

      {
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        if (sim_mode) {
          addr.sin_port = htons(50100 + command.robot_id);
          std::string address = "127.0.0.1";
          addr.sin_addr.s_addr = inet_addr(address.c_str());
        } else {
          addr.sin_port = htons(12345);
          std::string address = "192.168.20." + std::to_string(100 + command.robot_id);
          addr.sin_addr.s_addr = inet_addr(address.c_str());
        }

        sendto(
          sock, reinterpret_cast<uint8_t *>(&send_packet), 32, 0,
          reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
      }
    }
  }
};
}  // namespace crane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<crane::IbisSenderNode> ibis_sender_node =
    std::make_shared<crane::IbisSenderNode>(options);

  exe.add_node(ibis_sender_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
