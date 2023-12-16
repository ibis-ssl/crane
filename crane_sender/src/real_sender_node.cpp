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

  WorldModelWrapper::SharedPtr world_model;

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
      if (msg.is_yellow) {
        command.target_velocity.x *= -1;
        command.target_velocity.y *= -1;
        command.target_velocity.theta *= -1;
        if (not command.target_theta.empty()) {
          command.target_theta.front() *= -1;
        }
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
      packet.VISION_GLOBAL_THETA = command.current_pose.theta;

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
      //      std::cout << "id( " << command.robot_id << " ) is available: " << is_id_available
      //                << std::endl;
      // キーパーEN
      // 0 or 1

      packet.LOCAL_KEEPER_MODE_ENABLE = command.local_goalie_enable;

      packet.CHECK = check;

      RobotCommandSerialized serialized_packet(packet);
      sock = socket(AF_INET, SOCK_DGRAM, 0);
      // cspell: ignore BINDTODEVICE
      setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, opt, 4);
      addr.sin_family = AF_INET;
      addr.sin_port = htons(12345);
      std::string address = "192.168.20." + std::to_string(100 + command.robot_id);
      addr.sin_addr.s_addr = inet_addr(address.c_str());

      for (int i = 0; i < static_cast<int>(RobotCommandSerialized::Address::SIZE); ++i) {
        send_packet[i] = serialized_packet.data[i];
      }

      if (command.robot_id == debug_id) {
        //        printf(
        //          "ID=%d Vx=%.3f Vy=%.3f theta=%.3f", command.robot_id, command.target_velocity.x,
        //          command.target_velocity.y, target_theta);
        //        printf(
        //          " vision=%.3f kick=%.2f chip=%d Dri=%.2f", command.current_pose.theta,
        //          kick_power_send / 255.f, static_cast<int>(command.chip_enable),
        //          dribble_power_send / 255.f);
        //        printf(" keeper=%d check=%d", static_cast<int>(keeper_EN), static_cast<int>(check));
        //        printf("\n");

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
