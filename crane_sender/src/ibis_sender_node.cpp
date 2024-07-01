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

#include <boost/asio.hpp>
#include <class_loader/visibility_control.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "crane_sender/robot_packet.h"
#include "crane_sender/sender_base.hpp"

namespace crane
{
class RobotCommandSender
{
public:
  explicit RobotCommandSender(uint8_t robot_id, bool sim_mode)
  : io_service(), socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0))
  {
    boost::asio::ip::udp::resolver resolver(io_service);
    endpoint = [&]() -> boost::asio::ip::udp::endpoint {
      if (sim_mode) {
        std::string host = "localhost";
        boost::asio::ip::udp::resolver::query query(host, std::to_string(50100 + robot_id));
        return *resolver.resolve(query);
      } else {
        std::string host = "192.168.20." + std::to_string(100 + robot_id);
        boost::asio::ip::udp::resolver::query query(host, "12345");
        return *resolver.resolve(query);
      }
    }();
  }

  RobotCommandSerializedV2 send(RobotCommandV2 packet)
  {
    if (++check > 200) {
      check = 0;
    }

    packet.check_counter = check;
    RobotCommandSerializedV2 serialized_packet(packet);

    uint8_t send_packet[64] = {};
    for (int i = 0; i < 64; ++i) {
      send_packet[i] = serialized_packet.data[i];
    }

    socket.send_to(boost::asio::buffer(send_packet), endpoint);

    return serialized_packet;
  }

protected:
  boost::asio::io_service io_service;

  boost::asio::ip::udp::socket socket;

  boost::asio::ip::udp::endpoint endpoint;

  int check = 0;
};

class IbisSenderNode : public SenderBase
{
private:
  int debug_id;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  WorldModelWrapper::SharedPtr world_model;

  std::array<std::shared_ptr<RobotCommandSender>, 20> senders;

  bool sim_mode;

public:
  CLASS_LOADER_PUBLIC
  explicit IbisSenderNode(const rclcpp::NodeOptions & options) : SenderBase("ibis_sender", options)
  {
    declare_parameter("debug_id", -1);
    get_parameter("debug_id", debug_id);

    declare_parameter("sim_mode", true);
    get_parameter("sim_mode", sim_mode);
    parameter_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(this);
    parameter_callback_handle =
      parameter_subscriber->add_parameter_callback("debug_id", [&](const rclcpp::Parameter & p) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          debug_id = p.as_int();
        } else {
          std::cout << "debug_id is not integer" << std::endl;
        }
      });

    //    declare_parameter("interface", "enp4s0");
    //    get_parameter("interface", interface);
    //
    //    declare_parameter("sim", false);
    //    get_parameter("sim", sim_mode);
    //    if (sim_mode) {
    //      interface = "lo";
    //    }

    for (std::size_t i = 0; i < senders.size(); i++) {
      senders[i] = std::make_shared<RobotCommandSender>(i, sim_mode);
    }

    world_model = std::make_shared<WorldModelWrapper>(*this);

    std::cout << "start" << std::endl;
  }

  void sendCommands(const crane_msgs::msg::RobotCommands & msg) override
  {
    if (not world_model->hasUpdated()) {
      return;
    }

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

      RobotCommandV2 packet;
      packet.header = 0x00;
      packet.check_counter = 0;
      packet.vision_global_x = command.current_pose.x;
      packet.vision_global_y = command.current_pose.y;
      packet.vision_global_theta = command.current_pose.theta;
      packet.is_vision_available = [&]() -> bool {
        std::vector<uint8_t> available_ids = world_model->ours.getAvailableRobotIds();
        return std::count(available_ids.begin(), available_ids.end(), command.robot_id) == 1;
      }();
      packet.target_global_theta = [&]() -> float {
        if (not command.target_theta.empty()) {
          return normalize_angle(command.target_theta.front());
        } else {
          return 0.f;
        }
      }();
      packet.kick_power = std::clamp(command.kick_power, 0.f, 1.f);
      packet.dribble_power = std::clamp(command.dribble_power, 0.f, 1.f);
      packet.enable_chip = command.chip_enable;
      packet.lift_dribbler = command.lift_up_dribbler_flag;
      packet.stop_emergency = command.stop_flag;
      packet.speed_limit = command.target_velocity.x;  // TODO(Hans) 速度制限をちゃんとmsgに入れる
      packet.omega_limit = 10.f;  // TODO(Hans) 角速度制限をちゃんとmsgに入れる
      packet.prioritize_move = true;
      packet.prioritize_accurate_acceleration = true;

      packet.control_mode = RobotCommandV2::ControlMode::POSITION_TARGET_MODE;
      packet.position_target_mode_args = new PositionTargetModeArgs();
      packet.position_target_mode_args->target_global_x = [&]() -> float {
        if (not command.target_x.empty()) {
          return command.target_x.front();
        } else {
          return 0.f;
        }
      }();
      packet.position_target_mode_args->target_global_y = [&]() -> float {
        if (not command.target_y.empty()) {
          return command.target_y.front();
        } else {
          return 0.f;
        }
      }();
      packet.position_target_mode_args->speed_limit_at_target =
        command.local_planner_config.terminal_velocity;
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
