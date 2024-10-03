// Copyright (c) 2024 ibis-ssl
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

#include "crane_sender/parameter_with_event.hpp"
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
        std::cout << "made commander for " << host << ":" << 50100 + robot_id << std::endl;
        return *resolver.resolve(query);
      } else {
        std::string host = "192.168.20." + std::to_string(100 + robot_id);
        boost::asio::ip::udp::resolver::query query(host, "12345");
        std::cout << "made commander for " << host << ":12345" << std::endl;
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

class IbisSenderNode : public SenderBase
{
private:
  int debug_id;

  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle;

  WorldModelWrapper::SharedPtr world_model;

  std::array<std::shared_ptr<RobotCommandSender>, 20> senders;

  bool sim_mode;

  rclcpp::Clock clock;

  std::array<PIDController, 20> vx_controllers;
  std::array<PIDController, 20> vy_controllers;

  ParameterWithEvent p_gain;
  ParameterWithEvent i_gain;
  ParameterWithEvent d_gain;

  double I_SATURATION = 0.0;

public:
  CLASS_LOADER_PUBLIC
  explicit IbisSenderNode(const rclcpp::NodeOptions & options)
  : SenderBase("ibis_sender", options),
    clock(RCL_ROS_TIME),
    p_gain("p_gain", *this),
    i_gain("i_gain", *this),
    d_gain("d_gain", *this)
  {
    declare_parameter("debug_id", -1);
    get_parameter("debug_id", debug_id);

    declare_parameter("p_gain", 4.0);
    p_gain.value = get_parameter("p_gain").as_double();
    declare_parameter("i_gain", 0.0);
    i_gain.value = get_parameter("i_gain").as_double();
    declare_parameter("d_gain", 0.0);
    d_gain.value = get_parameter("d_gain").as_double();

    declare_parameter("i_saturation", I_SATURATION);
    I_SATURATION = get_parameter("i_saturation").as_double();

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

    p_gain.callback = [&](double value) {
      for (auto & controller : vx_controllers) {
        controller.setGain(value, i_gain.getValue(), d_gain.getValue());
      }
      for (auto & controller : vy_controllers) {
        controller.setGain(value, i_gain.getValue(), d_gain.getValue());
      }
    };

    i_gain.callback = [&](double value) {
      for (auto & controller : vx_controllers) {
        controller.setGain(p_gain.getValue(), value, d_gain.getValue());
      }
      for (auto & controller : vy_controllers) {
        controller.setGain(p_gain.getValue(), value, d_gain.getValue());
      }
    };

    d_gain.callback = [&](double value) {
      for (auto & controller : vx_controllers) {
        controller.setGain(p_gain.getValue(), i_gain.getValue(), value);
      }
      for (auto & controller : vy_controllers) {
        controller.setGain(p_gain.getValue(), i_gain.getValue(), value);
      }
    };

    for (auto & controller : vx_controllers) {
      controller.setGain(p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
    }

    for (auto & controller : vy_controllers) {
      controller.setGain(p_gain.getValue(), i_gain.getValue(), d_gain.getValue(), I_SATURATION);
    }

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

    auto now = clock.now();

    for (auto command : msg.robot_commands) {
      RobotCommandV2 packet;
      packet.header = 0x00;
      packet.check_counter = 0;
      packet.vision_global_pos[0] = command.current_pose.x;
      packet.vision_global_pos[1] = command.current_pose.y;
      packet.vision_global_theta = command.current_pose.theta;
      packet.is_vision_available = [&]() -> bool {
        std::vector<uint8_t> available_ids = world_model->ours.getAvailableRobotIds();
        return std::count(available_ids.begin(), available_ids.end(), command.robot_id) == 1;
      }();
      packet.latency_time_ms = current_latency_ms;  // TODO(Hans): ちゃんと計測する
      packet.target_global_theta = command.target_theta;
      packet.kick_power = [&]() {
        if (command.chip_enable) {
          return std::clamp(command.kick_power, 0.f, static_cast<float>(kick_power_limit_chip));
        } else {
          return std::clamp(command.kick_power, 0.f, static_cast<float>(kick_power_limit_straight));
        }
      }();
      packet.dribble_power = std::clamp(command.dribble_power, 0.f, 1.f);
      packet.enable_chip = command.chip_enable;
      packet.lift_dribbler = command.lift_up_dribbler_flag;
      packet.stop_emergency = command.stop_flag;
      packet.acceleration_limit = command.local_planner_config.max_acceleration;
      packet.linear_velocity_limit = command.local_planner_config.max_velocity;
      packet.angular_velocity_limit = command.local_planner_config.max_omega;
      packet.prioritize_move = true;
      packet.prioritize_accurate_acceleration = true;

      //      auto elapsed_time = now - world_model->getOurRobot(command.robot_id)->detection_stamp;
      packet.elapsed_time_ms_since_last_vision = 16.0;  // elapsed_time.nanoseconds() / 1e6;

      switch (command.control_mode) {
        case crane_msgs::msg::RobotCommand::POSITION_TARGET_MODE: {
          packet.control_mode = SIMPLE_VELOCITY_TARGET_MODE;
          if (not command.position_target_mode.empty()) {
            Velocity vel;
            vel << vx_controllers[command.robot_id].update(
              command.position_target_mode.front().target_x - command.current_pose.x, 1.f / 30.f),
              vy_controllers[command.robot_id].update(
                command.position_target_mode.front().target_y - command.current_pose.y, 1.f / 30.f);
            vel += vel.normalized() * command.local_planner_config.terminal_velocity;
            double max_velocity = command.local_planner_config.max_velocity;
            double current_velocity =
              std::hypot(command.current_velocity.x, command.current_velocity.y);
            max_velocity = std::min(
              max_velocity, current_velocity + command.local_planner_config.max_acceleration * 0.1);
            if (vel.norm() > max_velocity) {
              vel = vel.normalized() * max_velocity;
            }
            packet.mode_args.simple_velocity.target_global_vel[0] = vel.x();
            packet.mode_args.simple_velocity.target_global_vel[1] = vel.y();
          } else {
            packet.mode_args.simple_velocity.target_global_vel[0] = 0.0;
            packet.mode_args.simple_velocity.target_global_vel[1] = 0.0;
            std::cout << "empty position_target_mode" << std::endl;
          }
        } break;
        case crane_msgs::msg::RobotCommand::SIMPLE_VELOCITY_TARGET_MODE: {
          packet.control_mode = SIMPLE_VELOCITY_TARGET_MODE;
          if (not command.simple_velocity_target_mode.empty()) {
            packet.mode_args.simple_velocity.target_global_vel[0] =
              command.simple_velocity_target_mode.front().target_vx;
            packet.mode_args.simple_velocity.target_global_vel[1] =
              command.simple_velocity_target_mode.front().target_vy;
          } else {
            packet.mode_args.simple_velocity.target_global_vel[0] = 0.0;
            packet.mode_args.simple_velocity.target_global_vel[1] = 0.0;
            std::cout << "empty simple_velocity_target_mode" << std::endl;
          }
        } break;
        case crane_msgs::msg::RobotCommand::LOCAL_CAMERA_MODE: {
          packet.control_mode = SIMPLE_VELOCITY_TARGET_MODE;
          if (not command.local_camera_mode.empty()) {
            double vx = command.local_camera_mode.front().target_global_vx;
            double vy = command.local_camera_mode.front().target_global_vy;
            packet.mode_args.simple_velocity.target_global_vel[0] = vx;
            packet.mode_args.simple_velocity.target_global_vel[1] = vy;
          } else {
            packet.mode_args.simple_velocity.target_global_vel[0] = 0.0;
            packet.mode_args.simple_velocity.target_global_vel[1] = 0.0;
            std::cout << "empty local_camera_mode" << std::endl;
          }
        } break;
        default:
          packet.control_mode = SIMPLE_VELOCITY_TARGET_MODE;
          packet.mode_args.simple_velocity.target_global_vel[0] = 0.0;
          packet.mode_args.simple_velocity.target_global_vel[1] = 0.0;
          std::cout << "Invalid control mode" << std::endl;
          break;
      }
      if (command.stop_flag) {
        packet.mode_args.simple_velocity.target_global_vel[0] = 0.0;
        packet.mode_args.simple_velocity.target_global_vel[1] = 0.0;
      }
      senders[command.robot_id]->send(packet);
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
