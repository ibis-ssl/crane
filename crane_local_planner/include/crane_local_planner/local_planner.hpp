// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "RVO.h"
#include "crane_local_planner/visibility_control.h"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/robot_commands.hpp"
#include "crane_msgs/msg/world_model.hpp"

namespace crane
{
class LocalPlannerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit LocalPlannerComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("local_planner", options)
  {
    declare_parameter("enable_rvo", true);
    enable_rvo = get_parameter("enable_rvo").as_bool();

    float time_step = 1.0 / 60.0f;
    float neighbor_dist = 2.0f;
    size_t max_neighbors = 5;
    float time_horizon = 1.f;
    float time_horizon_obst = 1.f;
    float radius = 0.09f;
    float max_speed = 10.0f;
    rvo_sim = std::make_unique<RVO::RVOSimulator>(
      time_step, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst, radius, max_speed);

    world_model = std::make_shared<WorldModelWrapper>(*this);

    // TODO(HansRobo): add goal area as obstacles

    // TODO(HansRobo): add external area as obstacles
    // friend robots -> 0~19
    // enemy robots -> 20~39
    for (int i = 0; i < 40; i++) {
      rvo_sim->addAgent(RVO::Vector2(20.0f, 20.0f));
    }

    commnads_pub = this->create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
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

  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr commnads_pub;

  std::unique_ptr<RVO::RVOSimulator> rvo_sim;

  WorldModelWrapper::SharedPtr world_model;

  bool enable_rvo;
};

}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
