// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <functional>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "gridmap_planner.hpp"
#include "rvo2_planner.hpp"
#include "simple_planner.hpp"
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
    declare_parameter("planner", "gridmap");
    auto planner_str = get_parameter("planner").as_string();

    process_time_pub = create_publisher<std_msgs::msg::Float32>("process_time", 10);
    if (planner_str == "gridmap") {
      planner = std::make_shared<GridMapPlanner>(*this);
    } else if (planner_str == "simple") {
      planner = std::make_shared<SimplePlanner>(*this);
    } else if (planner_str == "rvo2") {
      planner = std::make_shared<RVO2Planner>(*this);
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown planner: %s", planner_str.c_str());
      throw std::runtime_error("Unknown planner: " + planner_str);
    }

    commands_pub = this->create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
    control_targets_sub = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/control_targets", 10,
      std::bind(&LocalPlannerComponent::callbackRobotCommands, this, std::placeholders::_1));
  }

  void callbackRobotCommands(const crane_msgs::msg::RobotCommands &);

private:
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr control_targets_sub;

  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr commands_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr process_time_pub;
  WorldModelWrapper::SharedPtr world_model;

  std::shared_ptr<crane::LocalPlannerBase> planner = nullptr;
};

}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
