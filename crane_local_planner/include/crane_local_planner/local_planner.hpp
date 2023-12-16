// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <crane_geometry/pid_controller.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "RVO.h"
#include "visibility_control.h"

namespace crane
{

struct Obstacle
{
  Point center;
  float radius;
};

class LocalPlannerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit LocalPlannerComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("local_planner", options)
  {
    declare_parameter("enable_rvo", true);
    enable_rvo = get_parameter("enable_rvo").as_bool();

    declare_parameter("rvo_time_step", RVO_TIME_STEP);
    RVO_TIME_STEP = get_parameter("rvo_time_step").as_double();
    declare_parameter("rvo_neighbor_dist", RVO_NEIGHBOR_DIST);
    RVO_NEIGHBOR_DIST = get_parameter("rvo_neighbor_dist").as_double();
    declare_parameter("rvo_max_neighbors", RVO_MAX_NEIGHBORS);
    RVO_MAX_NEIGHBORS = get_parameter("rvo_max_neighbors").as_int();
    declare_parameter("rvo_time_horizon", RVO_TIME_HORIZON);
    RVO_TIME_HORIZON = get_parameter("rvo_time_horizon").as_double();
    // cspell: ignore OBST
    declare_parameter("rvo_time_horizon_obst", RVO_TIME_HORIZON_OBST);
    RVO_TIME_HORIZON_OBST = get_parameter("rvo_time_horizon_obst").as_double();
    declare_parameter("rvo_radius", RVO_RADIUS);
    RVO_RADIUS = get_parameter("rvo_radius").as_double();
    declare_parameter("rvo_max_speed", RVO_MAX_SPEED);
    RVO_MAX_SPEED = get_parameter("rvo_max_speed").as_double();
    declare_parameter("rvo_trapezoidal_max_acc", RVO_TRAPEZOIDAL_MAX_ACC);
    RVO_TRAPEZOIDAL_MAX_ACC = get_parameter("rvo_trapezoidal_max_acc").as_double();
    declare_parameter("rvo_trapezoidal_frame_rate", RVO_TRAPEZOIDAL_FRAME_RATE);
    RVO_TRAPEZOIDAL_FRAME_RATE = get_parameter("rvo_trapezoidal_frame_rate").as_double();
    declare_parameter("rvo_trapezoidal_max_speed", RVO_TRAPEZOIDAL_MAX_SPEED);
    RVO_TRAPEZOIDAL_MAX_SPEED = get_parameter("rvo_trapezoidal_max_speed").as_double();
    declare_parameter("non_rvo_max_vel", NON_RVO_MAX_VEL);
    NON_RVO_MAX_VEL = get_parameter("non_rvo_max_vel").as_double();

    declare_parameter("non_rvo_p_gain", NON_RVO_P_GAIN);
    NON_RVO_P_GAIN = get_parameter("non_rvo_p_gain").as_double();
    declare_parameter("non_rvo_i_gain", NON_RVO_I_GAIN);
    NON_RVO_I_GAIN = get_parameter("non_rvo_i_gain").as_double();
    declare_parameter("non_rvo_d_gain", NON_RVO_D_GAIN);
    NON_RVO_D_GAIN = get_parameter("non_rvo_d_gain").as_double();

    for (auto & controller : vx_controllers) {
      controller.setGain(NON_RVO_P_GAIN, NON_RVO_I_GAIN, NON_RVO_D_GAIN);
    }

    for (auto & controller : vy_controllers) {
      controller.setGain(NON_RVO_P_GAIN, NON_RVO_I_GAIN, NON_RVO_D_GAIN);
    }

    rvo_sim = std::make_unique<RVO::RVOSimulator>(
      RVO_TIME_STEP, RVO_NEIGHBOR_DIST, RVO_MAX_NEIGHBORS, RVO_TIME_HORIZON, RVO_TIME_HORIZON_OBST,
      RVO_RADIUS, RVO_MAX_SPEED);

    world_model = std::make_shared<WorldModelWrapper>(*this);

    // TODO(HansRobo): add goal area as obstacles

    // TODO(HansRobo): add external area as obstacles
    // friend robots -> 0~19
    // enemy robots -> 20~39
    // ball 40
    for (int i = 0; i < 41; i++) {
      rvo_sim->addAgent(RVO::Vector2(20.0f, 20.0f));
    }

    commands_pub = this->create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
    control_targets_sub = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/control_targets", 10,
      std::bind(&LocalPlannerComponent::callbackControlTarget, this, std::placeholders::_1));
  }

  void reflectWorldToRVOSim(const crane_msgs::msg::RobotCommands &);

  crane_msgs::msg::RobotCommands extractRobotCommandsFromRVOSim(
    const crane_msgs::msg::RobotCommands &);

  void callbackControlTarget(const crane_msgs::msg::RobotCommands &);

private:
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr control_targets_sub;

  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr commands_pub;

  std::unique_ptr<RVO::RVOSimulator> rvo_sim;

  WorldModelWrapper::SharedPtr world_model;

  bool enable_rvo;

  std::array<PIDController, 20> vx_controllers;
  std::array<PIDController, 20> vy_controllers;

  float RVO_TIME_STEP = 1.0 / 60.0f;
  float RVO_NEIGHBOR_DIST = 2.0f;
  int RVO_MAX_NEIGHBORS = 5;
  float RVO_TIME_HORIZON = 1.f;
  float RVO_TIME_HORIZON_OBST = 1.f;
  float RVO_RADIUS = 0.09f;
  float RVO_MAX_SPEED = 10.0f;

  float RVO_TRAPEZOIDAL_MAX_ACC = 8.0;
  float RVO_TRAPEZOIDAL_FRAME_RATE = 60;
  float RVO_TRAPEZOIDAL_MAX_SPEED = 4.0;

  double NON_RVO_MAX_VEL = 4.0;
  double NON_RVO_P_GAIN = 4.0;
  double NON_RVO_I_GAIN = 0.0;
  double NON_RVO_D_GAIN = 0.0;
};

}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
