// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <crane_basics/diagnosed_publisher.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <functional>
// #include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

// #include "gridmap_planner.hpp"
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

struct KickPowerCalculator
{
private:
  std::vector<double> kick_power_array;
  std::vector<double> kick_speed_array;

public:
  auto initialize(rclcpp::Node & node) -> void
  {
    node.declare_parameter<std::vector<double>>("kick_power_array", {});
    kick_power_array = node.get_parameter("kick_power_array").as_double_array();
    node.declare_parameter<std::vector<double>>("kick_speed_array", {});
    kick_speed_array = node.get_parameter("kick_speed_array").as_double_array();
  }

  auto getKickPower(const crane_msgs::msg::RobotCommand & command) const -> double
  {
    if (not command.local_planner_config.kick_power_override) {
      return command.kick_power;
    }

    if (kick_speed_array.size() == kick_power_array.size() && kick_power_array.size() <= 1) {
      std::stringstream what;
      what << "kick_speed_arrayとkick_power_arrayのサイズは等しく、2以上でなければいけません。";
      what << "size(kick_speed_array): " << static_cast<int>(kick_speed_array.size());
      what << ", size(kick_power_array): " << static_cast<int>(kick_power_array.size());
      throw std::runtime_error(what.str());
    }

    double kick_speed = command.local_planner_config.target_kick_ball_speed;
    // kick_speedが最小値より小さい場合
    if (kick_speed <= kick_speed_array[0]) {
      return kick_power_array[0];
    }

    // kick_speedが最大値より大きい場合
    if (kick_speed >= kick_speed_array.back()) {
      return kick_power_array.back();
    }

    // 線形補間
    int idx = 0;
    for (size_t i = 1; i < kick_speed_array.size(); ++i) {
      if (kick_speed_array[i] > kick_speed) {
        idx = i - 1;
        break;
      }
    }
    double kick_power_diff = kick_power_array[idx + 1] - kick_power_array[idx];
    double kick_speed_diff = kick_speed_array[idx + 1] - kick_speed_array[idx];
    double kick_power = kick_power_array[idx] +
                        kick_power_diff * (kick_speed - kick_speed_array[idx]) / kick_speed_diff;
    return kick_power;
  }
};

class LocalPlannerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit LocalPlannerComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("local_planner", options), commands_pub(this, "/robot_commands", 10, 50., 70.)
  {
    declare_parameter("planner", "rvo2");
    auto planner_str = get_parameter("planner").as_string();

    kick_power_calculator.initialize(*this);

    crane::CraneVisualizerBuffer::activate(*this);

    process_time_pub = create_publisher<std_msgs::msg::Float32>("process_time", 10);
    // if (planner_str == "gridmap") {
    //   planner = std::make_shared<GridMapPlanner>(*this);
    // }
    if (planner_str == "simple") {
      planner = std::make_shared<SimplePlanner>(*this);
    } else if (planner_str == "rvo2") {
      planner = std::make_shared<RVO2Planner>(*this);
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown planner: %s", planner_str.c_str());
      throw std::runtime_error("Unknown planner: " + planner_str);
    }

    // 練習用モードの設定
    bool half_court_practice_mode = false;
    bool half_court_is_positive_side = true;  // 使用している半面がポジティブ側かどうか
    declare_parameter("half_court_practice_mode", half_court_practice_mode);
    get_parameter("half_court_practice_mode", half_court_practice_mode);
    declare_parameter("half_court_is_positive_side", half_court_is_positive_side);
    get_parameter("half_court_is_positive_side", half_court_is_positive_side);

    if (half_court_practice_mode) {
      theta_offset = -M_PI / 2.;
    } else {
      theta_offset = 0.;
    }

    control_targets_sub = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/control_targets", 10,
      std::bind(&LocalPlannerComponent::callbackRobotCommands, this, std::placeholders::_1));
  }

  auto callbackRobotCommands(const crane_msgs::msg::RobotCommands &) -> void;

private:
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr control_targets_sub;

  DiagnosedPublisher<crane_msgs::msg::RobotCommands> commands_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr process_time_pub;

  std::shared_ptr<crane::LocalPlannerBase> planner = nullptr;

  double theta_offset = 0.;

  KickPowerCalculator kick_power_calculator;
};

}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
