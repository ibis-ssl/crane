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

#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "class_loader/visibility_control.hpp"
#include "crane_msgs/msg/robot_commands.hpp"
#include "crane_sender/sender_base.hpp"

int check;
int sock;
struct sockaddr_in addr;

const char * opt = "enx00e04c696e2b";

namespace crane
{
class RealSenderNode : public SenderBase
{
public:
  CLASS_LOADER_PUBLIC
  explicit RealSenderNode(const rclcpp::NodeOptions & options) : SenderBase("real_sender", options)
  {
    std::cout << "start" << std::endl;
  }
  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    // TODO(okada_tech) : send commands to robots

    uint8_t vel_surge_send_high, vel_surge_send_low, vel_sway_send_high, vel_sway_send_low,
      vel_angular_vision_send_high, vel_angular_vision_send_low;
    uint8_t vel_angular_consai_send_high, vel_angular_consai_send_low, kick_power_send,
      dribble_power_send, keeper_EN;
    uint8_t send_packet[12];

    constexpr double MAX_VEL_SURGE = 7.0;  // m/s
    constexpr double MAX_VEL_SWAY = 7.0;   // m/s
    constexpr double MAX_VEL_ANGULAR = 2.0 * M_PI;

    for (auto command : msg.robot_commands) {
      // vel_surge
      //  -7 ~ 7 -> 0 ~ 32767 ~ 65534
      //  -7 -> 0
      //  0 -> 32767
      //  7 -> 65534
      uint16_t vel_surge_send = static_cast<int>(
        32767 * static_cast<float>(command.target_velocity.x / MAX_VEL_SURGE) + 32767);
      vel_surge_send_low = vel_surge_send & 0x00FF;
      vel_surge_send_high = (vel_surge_send & 0xFF00) >> 8;

      // vel_sway
      // -7 ~ 7 -> 0 ~ 32767 ~ 65534
      // -7 -> 0
      // 0 -> 32767
      // 7 -> 65534
      uint16_t vel_sway_send = static_cast<int>(
        32767 * static_cast<float>(command.target_velocity.y / MAX_VEL_SWAY) + 32767);
      vel_sway_send_low = vel_sway_send & 0x00FF;
      vel_sway_send_high = (vel_sway_send & 0xFF00) >> 8;

      // 目標角度
      // -pi ~ pi -> 0 ~ 32767 ~ 65534
      // -pi -> 0
      // 0 -> 32767
      // pi -> 65534
      float vel_angular_consai = command.target_velocity.theta;
      if (fabs(vel_angular_consai) > M_PI) {
        while (vel_angular_consai > M_PI) {
          vel_angular_consai -= 2.0f * M_PI;
        }
        while (vel_angular_consai < -M_PI) {
          vel_angular_consai += 2.0f * M_PI;
        }
      }

      // -pi ~ pi -> 0 ~ 32767 ~ 65534
      uint16_t vel_angular_consai_send =
        static_cast<int>(32767 * static_cast<float>(vel_angular_consai / M_PI) + 32767);
      vel_angular_consai_send_low = vel_angular_consai_send & 0x00FF;
      vel_angular_consai_send_high = (vel_angular_consai_send & 0xFF00) >> 8;

      // Vision角度
      // -pi ~ pi -> 0 ~ 32767 ~ 65534
      // pi -> 0
      // 0 -> 32767
      // pi -> 65534
      float vel_angular_vision = command.current_pose.theta;

      if (fabs(vel_angular_vision) > M_PI) {
        vel_angular_vision = copysign(M_PI, vel_angular_vision);
      }
      // -pi ~ pi -> 0 ~ 32767 ~ 65534
      uint16_t vel_angular_vision_send =
        static_cast<int>(32767 * static_cast<float>(vel_angular_vision / M_PI) + 32767);
      vel_angular_vision_send_low = vel_angular_vision_send & 0x00FF;
      vel_angular_vision_send_high = (vel_angular_vision_send & 0xFF00) >> 8;

      // ドリブル
      // 0 ~ 1.0 -> 0 ~ 20
      float dribble_power = 0;

      if (command.dribble_power > 0) {
        dribble_power = command.dribble_power;
        if (dribble_power > 1.0) {
          dribble_power = 1.0;
        } else if (dribble_power < 0) {
          dribble_power = 0.0;
        }
        dribble_power_send = static_cast<int>(round(20 * dribble_power));
      } else {
        dribble_power_send = 0;
      }

      // キック
      // 0 ~ 1.0 -> 0 ~ 20
      // チップキック有効の場合　0 ~ 1.0 -> 100 ~ 120
      float kick_power = 0;
      if (command.kick_power > 0) {
        kick_power = command.kick_power;
        if (kick_power > 1.0) {
          kick_power = 1.0;
        } else if (kick_power < 0) {
          kick_power = 0;
        }
        if (command.chip_enable) {
          kick_power_send = static_cast<int>((round(20 * kick_power) + 100));
        } else {
          kick_power_send = static_cast<int>(round(20 * kick_power));
        }
      } else {
        kick_power_send = 0;
      }

      // キーパーEN
      // 0 or 1
      keeper_EN = command.local_goalie_enable;

      switch (command.robot_id) {
        case 0:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.100");
          break;

        case 1:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.101");
          break;

        case 2:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.102");
          break;

        case 3:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.103");
          break;

        case 4:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.104");
          break;

        case 5:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.105");
          break;

        case 6:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.106");
          break;

        case 7:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.107");
          break;

        case 8:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.108");
          break;

        case 9:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.109");
          break;

        case 10:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.110");
          break;

        case 11:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.111");
          break;

        case 12:
          sock = socket(AF_INET, SOCK_DGRAM, 0);
          setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
          addr.sin_family = AF_INET;
          addr.sin_port = htons(12345);
          addr.sin_addr.s_addr = inet_addr("192.168.20.112");
          break;
      }

      send_packet[0] = static_cast<uint8_t>(vel_surge_send_high);
      send_packet[1] = static_cast<uint8_t>(vel_surge_send_low);
      send_packet[2] = static_cast<uint8_t>(vel_sway_send_high);
      send_packet[3] = static_cast<uint8_t>(vel_sway_send_low);
      send_packet[4] = static_cast<uint8_t>(vel_angular_vision_send_high);
      send_packet[5] = static_cast<uint8_t>(vel_angular_vision_send_low);
      send_packet[6] = static_cast<uint8_t>(vel_angular_consai_send_high);
      send_packet[7] = static_cast<uint8_t>(vel_angular_consai_send_low);
      send_packet[8] = static_cast<uint8_t>(kick_power_send);
      send_packet[9] = static_cast<uint8_t>(dribble_power_send);
      send_packet[10] = static_cast<uint8_t>(keeper_EN);
      send_packet[11] = static_cast<uint8_t>(check);

      if (command.robot_id == 1) {
        printf(
          "ID=%d Vx=%.3f Vy=%.3f theta=%.3f", command.robot_id, command.target_velocity.x, command.target_velocity.y,
          vel_angular_consai);
        printf(
          " vision=%.3f kick=%.2f chip=%d Dri=%.2f", vel_angular_vision, kick_power,
          static_cast<int>(command.chip_enable), dribble_power);
        printf(" keeper=%d check=%d", static_cast<int>(keeper_EN), static_cast<int>(check));
        printf("\n");

        printf(
          "=%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x =%x", static_cast<int>(send_packet[0]),
          send_packet[1], send_packet[2], send_packet[3], send_packet[4], send_packet[5],
          send_packet[6], send_packet[7], send_packet[8], send_packet[9], send_packet[10],
          send_packet[11]);
        printf("\n");
        printf("\n");
      }
      check++;
      if (check > 200) {
        check = 0;
      }

      sendto(
        sock, reinterpret_cast<uint8_t *>(&send_packet), 12, 0,
        reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
      close(sock);
    }
  }
};
}  // namespace crane

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<crane::RealSenderNode> real_sender_node =
    std::make_shared<crane::RealSenderNode>(options);

  exe.add_node(real_sender_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
