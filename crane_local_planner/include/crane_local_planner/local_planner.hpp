// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <crane_comm/diagnosed_publisher.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_physics/kicker_model.hpp>
#include <functional>
// #include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

// #include "gridmap_planner.hpp"
#include "modern_orca_planner.hpp"
#include "rvo2_planner.hpp"
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
  // KickerModel統合（すべてのキック計算用）
  std::shared_ptr<KickerModel> kicker_model;

public:
  auto initialize(rclcpp::Node & node) -> void
  {
    // KickerModel初期化（YAML設定ファイルから読み込み）
    node.declare_parameter<std::string>("kicker_physics_config", "");
    std::string kicker_config_path = node.get_parameter("kicker_physics_config").as_string();

    try {
      if (!kicker_config_path.empty()) {
        // YAML設定ファイルからKickerModelを作成
        kicker_model = createKickerModelFromYAML(kicker_config_path);
        RCLCPP_INFO(
          node.get_logger(), "KickerModel initialized from YAML: %s", kicker_config_path.c_str());
      } else {
        // デフォルト設定でKickerModelを作成
        kicker_model = createDefaultKickerModel();
        RCLCPP_INFO(node.get_logger(), "KickerModel initialized with default values");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node.get_logger(), "KickerModel initialization failed: %s", e.what());
      throw std::runtime_error("Failed to initialize KickerModel: " + std::string(e.what()));
    }
  }

  auto getKickPower(const crane_msgs::msg::RobotCommand & command) const -> double
  {
    if (not command.local_planner_config.kick_power_override) {
      return command.kick_power;
    }

    // KickerModelを使用（必須）
    if (!kicker_model) {
      throw std::runtime_error("KickerModel is not initialized");
    }

    try {
      if (command.chip_enable) {
        return kicker_model->calculateChipKickPower(
          command.local_planner_config.target_chip_distance);
      } else {
        return kicker_model->calculateStraightKickPower(
          command.local_planner_config.target_kick_ball_speed);
      }
    } catch (const std::exception & e) {
      throw std::runtime_error("KickerModel calculation failed: " + std::string(e.what()));
    }
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
    if (planner_str == "rvo2") {
      planner = std::make_shared<RVO2Planner>(*this);
    } else if (planner_str == "modern_orca") {
      planner = std::make_shared<ModernORCAPlanner>(*this);
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

protected:
  static auto resolveMaxAccelerationFactors(
    crane_msgs::msg::RobotCommand & command, const float default_max_acceleration = 5.0) -> double;

  static auto resolveMaxVelocityFactors(
    crane_msgs::msg::RobotCommand & command, const float default_max_velocity = 5.0) -> double;

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
