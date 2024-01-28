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

#include "gridmap_planner.hpp"
#include "rvo_planner.hpp"
#include "simple_planner.hpp"
#include "simple_avoid_planner.hpp"
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
    declare_parameter("planner", "simple");
    auto planner_str = get_parameter("planner").as_string();

    if (planner_str == "simple") {
      simple_planner = std::make_shared<SimplePlanner>(*this);
      calculate_control_target =
        [this](const crane_msgs::msg::RobotCommands & msg) -> crane_msgs::msg::RobotCommands {
        return simple_planner->calculateControlTarget(msg, world_model);
      };
    } else if (planner_str == "simple_avoid") {
      simple_avoid_planner = std::make_shared<SimpleAvoidPlanner>(*this);
      calculate_control_target =
        [this](const crane_msgs::msg::RobotCommands & msg) -> crane_msgs::msg::RobotCommands {
        return simple_avoid_planner->calculateControlTarget(msg, world_model);
      };
    } else if (planner_str == "rvo") {
      rvo_planner = std::make_shared<RVOPlanner>(*this);
      calculate_control_target =
        [this](const crane_msgs::msg::RobotCommands & msg) -> crane_msgs::msg::RobotCommands {
        return rvo_planner->calculateControlTarget(msg, world_model);
      };
    } else if (planner_str == "gridmap") {
      gridmap_planner = std::make_shared<GridMapPlanner>(*this);
      calculate_control_target =
        [this](const crane_msgs::msg::RobotCommands & msg) -> crane_msgs::msg::RobotCommands {
        return gridmap_planner->calculateControlTarget(msg, world_model);
      };
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown planner: %s", planner_str.c_str());
      throw std::runtime_error("Unknown planner: " + planner_str);
    }

    world_model = std::make_shared<WorldModelWrapper>(*this);

    commands_pub = this->create_publisher<crane_msgs::msg::RobotCommands>("/robot_commands", 10);
    control_targets_sub = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/control_targets", 10,
      std::bind(&LocalPlannerComponent::callbackControlTarget, this, std::placeholders::_1));
  }

  void callbackControlTarget(const crane_msgs::msg::RobotCommands &);

private:
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr control_targets_sub;

  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr commands_pub;

  WorldModelWrapper::SharedPtr world_model;

  std::function<crane_msgs::msg::RobotCommands(const crane_msgs::msg::RobotCommands &)>
    calculate_control_target;

  std::shared_ptr<SimplePlanner> simple_planner = nullptr;
  std::shared_ptr<SimpleAvoidPlanner> simple_avoid_planner = nullptr;
  std::shared_ptr<RVOPlanner> rvo_planner = nullptr;
  std::shared_ptr<GridMapPlanner> gridmap_planner = nullptr;
};

}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
