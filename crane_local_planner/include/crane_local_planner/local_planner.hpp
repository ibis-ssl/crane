// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_

#include <crane_comm/diagnosed_publisher.hpp>
#include <crane_comm/diagnostic_helper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_physics/kicker_model.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>

#include "rvo2_planner.hpp"
#include "visibility_control.h"

namespace crane
{

class LocalPlannerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit LocalPlannerComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("local_planner", options),
    commands_pub(this, "/robot_commands", 10, 50., 70.),
    diagnostic_helper_(
      this, "local_planner", "local_planner/path_planning", this,
      &LocalPlannerComponent::updateDiagnostics)
  {
    declare_parameter("planner", "rvo2");
    auto planner_str = get_parameter("planner").as_string();

    // KickerModel初期化（YAML設定ファイルから読み込み）
    declare_parameter<std::string>("kicker_physics_config", "");
    std::string kicker_config_path = get_parameter("kicker_physics_config").as_string();
    try {
      if (!kicker_config_path.empty()) {
        kicker_model_ = createKickerModelFromYAML(kicker_config_path);
        RCLCPP_INFO(
          get_logger(), "KickerModel initialized from YAML: %s", kicker_config_path.c_str());
      } else {
        kicker_model_ = createDefaultKickerModel();
        RCLCPP_INFO(get_logger(), "KickerModel initialized with default values");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "KickerModel initialization failed: %s", e.what());
      throw std::runtime_error("Failed to initialize KickerModel: " + std::string(e.what()));
    }

    crane::CraneVisualizerBuffer::activate(*this);

    process_time_pub = create_publisher<std_msgs::msg::Float32>("process_time", 10);
    if (planner_str == "rvo2") {
      planner = std::make_shared<RVO2Planner>(*this);
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown planner: %s", planner_str.c_str());
      throw std::runtime_error("Unknown planner: " + planner_str);
    }

    // 練習用モードの設定
    bool half_court_practice_mode = false;
    declare_parameter("half_court_practice_mode", half_court_practice_mode);
    get_parameter("half_court_practice_mode", half_court_practice_mode);

    if (half_court_practice_mode) {
      theta_offset = -M_PI / 2.;
    } else {
      theta_offset = 0.;
    }

    control_targets_sub = this->create_subscription<crane_msgs::msg::RobotCommands>(
      "/control_targets", 10,
      std::bind(&LocalPlannerComponent::callbackPositionCommands, this, std::placeholders::_1));
  }

  auto callbackPositionCommands(const crane_msgs::msg::RobotCommands &) -> void;

  auto updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) -> void;

private:
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr control_targets_sub;

  DiagnosedPublisher<crane_msgs::msg::RobotCommands> commands_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr process_time_pub;

  std::shared_ptr<crane::LocalPlannerBase> planner = nullptr;

  double theta_offset = 0.;

  std::shared_ptr<KickerModel> kicker_model_;

  DiagnosticHelper diagnostic_helper_;

  size_t dropped_command_count_last_cycle_ = 0;
  size_t dropped_command_count_total_ = 0;
  size_t planner_exception_count_last_cycle_ = 0;
  size_t planner_exception_count_total_ = 0;

  auto aggregateStates(const std::vector<crane_msgs::msg::NamedString> & planning_factors) const
    -> std::string;

  auto getKickPower(const crane_msgs::msg::RobotCommand & command) const -> double
  {
    if (not command.local_planner_config.kick_power_override) {
      return command.kick_power;
    }
    if (!kicker_model_) {
      throw std::runtime_error("KickerModel is not initialized");
    }
    try {
      if (command.chip_enable) {
        return kicker_model_->calculateChipKickPower(
          command.local_planner_config.target_chip_distance);
      } else {
        return kicker_model_->calculateStraightKickPower(
          command.local_planner_config.target_kick_ball_speed);
      }
    } catch (const std::exception & e) {
      throw std::runtime_error("KickerModel calculation failed: " + std::string(e.what()));
    }
  }
};

}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__LOCAL_PLANNER_HPP_
