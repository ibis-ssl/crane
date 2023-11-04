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
#include <crane_msgs/msg/robot_commands.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "sender_base.hpp"

int check;
int sock;
struct sockaddr_in addr;

const char * opt = "enx00e04c696e2b";

namespace crane
{
class RealSenderNode : public SenderBase
{
private:
  int debug_id;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

public:
  CLASS_LOADER_PUBLIC
  explicit RealSenderNode(const rclcpp::NodeOptions & options) : SenderBase("real_sender", options)
  {
    declare_parameter("debug_id", 1);
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

    std::cout << "start" << std::endl;
  }
  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    // TODO(okada_tech) : send commands to robots

    uint8_t send_packet[32] = {};

    constexpr double MAX_VEL_SURGE = 7.0;  // m/s
    constexpr double MAX_VEL_SWAY = 7.0;   // m/s
    constexpr double MAX_VEL_ANGULAR = 2.0 * M_PI;

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
      // vel_surge
      //  -7 ~ 7 -> 0 ~ 32767 ~ 65534
      // 取り敢えず横偏差をなくすためにy方向だけゲインを高めてみる
      if (std::abs(command.target_velocity.y) < 0.3) {
        //        command.target_velocity.y *= 8.f;
      }
      auto [vel_surge_low, vel_surge_high] = to_two_byte(command.target_velocity.x, MAX_VEL_SURGE);

      // vel_sway
      // -7 ~ 7 -> 0 ~ 32767 ~ 65534
      auto [vel_sway_low, vel_sway_high] = to_two_byte(command.target_velocity.y, MAX_VEL_SWAY);

      // 目標角度
      float target_theta = [&]() -> float {
        if (not command.target_theta.empty()) {
          return normalize_angle(command.target_theta.front());
        } else {
          return 0.f;
        }
      }();

      // -pi ~ pi -> 0 ~ 32767 ~ 65534
      auto [target_theta_low, target_theta_high] = to_two_byte(target_theta, M_PI);

      // Vision角度
      // -pi ~ pi -> 0 ~ 32767 ~ 65534
      float vision_theta = normalize_angle(command.current_pose.theta);

      // -pi ~ pi -> 0 ~ 32767 ~ 65534
      auto [vision_theta_low, vision_theta_high] = to_two_byte(vision_theta, M_PI);

      // ドリブル
      // 0 ~ 1.0 -> 0 ~ 20
      uint8_t dribble_power_send = [&]() {
        float dribble_power = 0;
        if (command.dribble_power > 0) {
          dribble_power = command.dribble_power;
          if (dribble_power > 1.0) {
            dribble_power = 1.0;
          } else if (dribble_power < 0) {
            dribble_power = 0.0;
          }
          return static_cast<int>(round(20 * dribble_power));
        } else {
          return 0;
        }
      }();

      // キック
      // 0 ~ 1.0 -> 0 ~ 20
      // チップキック有効の場合　0 ~ 1.0 -> 100 ~ 120
      uint8_t kick_power_send = [&]() {
        float kick_power = 0;
        if (command.kick_power > 0) {
          kick_power = command.kick_power;
          if (kick_power > 1.0) {
            kick_power = 1.0;
          } else if (kick_power < 0) {
            kick_power = 0;
          }
          if (command.chip_enable) {
            return static_cast<int>((round(20 * kick_power) + 100));
          } else {
            return static_cast<int>(round(20 * kick_power));
          }
        } else {
          return 0;
        }
      }();

      // キーパーEN
      // 0 or 1
      uint8_t keeper_EN = command.local_goalie_enable;

      // Vision位置
      //  -32.767 ~ 0 ~ 32.767 -> 0 ~ 32767 ~ 65534
      // meter -> mili meter
      auto [vision_x_low, vision_x_high] = to_two_byte(command.current_pose.x, 32.767);
      auto [vision_y_low, vision_y_high] = to_two_byte(command.current_pose.y, 32.767);

      //ボール座標
      auto [ball_x_low, ball_x_high] = to_two_byte(command.current_ball_x, 32.767);
      auto [ball_y_low, ball_y_high] = to_two_byte(command.current_ball_y, 32.767);

      // 目標座標
      float target_x = 0.f;
      float target_y = 0.f;
      bool enable_local_feedback = true;
      if (not command.target_x.empty()) {
        target_x = command.target_x.front();
      } else {
        enable_local_feedback = false;
      }
      if (not command.target_y.empty()) {
        target_y = command.target_y.front();
      } else {
        enable_local_feedback = false;
      }
      auto [target_x_low, target_x_high] = to_two_byte(target_x, 32.767);
      auto [target_y_low, target_y_high] = to_two_byte(target_y, 32.767);

      sock = socket(AF_INET, SOCK_DGRAM, 0);
      setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
      addr.sin_family = AF_INET;
      addr.sin_port = htons(12345);
      std::string address = "192.168.20." + std::to_string(100 + command.robot_id);
      addr.sin_addr.s_addr = inet_addr(address.c_str());

      send_packet[0] = static_cast<uint8_t>(vel_surge_high);
      send_packet[1] = static_cast<uint8_t>(vel_surge_low);
      send_packet[2] = static_cast<uint8_t>(vel_sway_high);
      send_packet[3] = static_cast<uint8_t>(vel_sway_low);
      send_packet[4] = static_cast<uint8_t>(vision_theta_high);
      send_packet[5] = static_cast<uint8_t>(vision_theta_low);
      send_packet[6] = static_cast<uint8_t>(target_theta_high);
      send_packet[7] = static_cast<uint8_t>(target_theta_low);
      send_packet[8] = static_cast<uint8_t>(kick_power_send);
      send_packet[9] = static_cast<uint8_t>(dribble_power_send);
      send_packet[10] = static_cast<uint8_t>(keeper_EN);
      send_packet[11] = static_cast<uint8_t>(ball_x_high);
      send_packet[12] = static_cast<uint8_t>(ball_x_low);
      send_packet[13] = static_cast<uint8_t>(ball_y_high);
      send_packet[14] = static_cast<uint8_t>(ball_y_low);
      send_packet[15] = static_cast<uint8_t>(vision_x_high);
      send_packet[16] = static_cast<uint8_t>(vision_x_low);
      send_packet[17] = static_cast<uint8_t>(vision_y_high);
      send_packet[18] = static_cast<uint8_t>(vision_y_low);
      send_packet[19] = static_cast<uint8_t>(target_x_high);
      send_packet[20] = static_cast<uint8_t>(target_x_low);
      send_packet[21] = static_cast<uint8_t>(target_y_high);
      send_packet[22] = static_cast<uint8_t>(target_y_low);
      send_packet[23] = static_cast<uint8_t>(enable_local_feedback);
      send_packet[24] = static_cast<uint8_t>(check);

      if (command.robot_id == debug_id) {
        printf(
          "ID=%d Vx=%.3f Vy=%.3f theta=%.3f", command.robot_id, command.target_velocity.x,
          command.target_velocity.y, target_theta);
        printf(
          " vision=%.3f kick=%.2f chip=%d Dri=%.2f", command.current_pose.theta,
          kick_power_send / 255.f, static_cast<int>(command.chip_enable),
          dribble_power_send / 255.f);
        printf(" keeper=%d check=%d", static_cast<int>(keeper_EN), static_cast<int>(check));
        printf("\n");

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

      sendto(
        sock, reinterpret_cast<uint8_t *>(&send_packet), 32, 0,
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
