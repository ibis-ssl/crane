// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crane_planner_plugins/test_planner.hpp>
#include <filesystem>

namespace crane
{
TestPlanner::TestPlanner(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
: PlannerBase("Test", world_model), topics_interface(node.get_node_topics_interface())
{
  config_file_path =
    (std::filesystem::path(ament_index_cpp::get_package_share_directory("crane_planner_plugins")) /
     "config" / "test_planner.yaml")
      .string();
  if (not loadConfigFromFile(config_file_path)) {
    RCLCPP_WARN(
      rclcpp::get_logger("TestPlanner"), "設定の読込に失敗: %s", config_file_path.c_str());
  }
  reload_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
    topics_interface, "/test_planner/reload", rclcpp::QoS(1),
    [this](std_msgs::msg::Empty::ConstSharedPtr) { reload_requested = true; });
}

std::pair<PlannerBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
TestPlanner::calculateRobotCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  if (robots.empty()) {
    return {PlannerBase::Status::RUNNING, robot_commands};
  }

  // リロード要求に対応
  if (reload_requested) {
    if (not loadConfigFromFile(config_file_path)) {
      RCLCPP_WARN(
        rclcpp::get_logger("TestPlanner"), "設定の読込に失敗: %s", config_file_path.c_str());
    }
    current_index = 0;
    sleep_until.reset();
    reload_requested = false;
  }

  auto & wp = waypoints.at(current_index);
  // 毎フレームfactorsをクリア
  command->clearMaxVelocityFactors().clearMaxAccelerationFactors();
  applyLegLimits(*command, wp);
  command->setTargetPosition(wp.pos);
  // 角度が設定されている場合は角度も設定
  if (wp.theta.has_value()) {
    command->setTargetTheta(wp.theta.value());
  }

  auto now = rclcpp::Clock().now();
  if (command->getTargetDistance() < 0.05) {
    if (not sleep_until.has_value()) {
      std::cout << "[TestPlanner] 到着" << std::endl;
      sleep_until = now + rclcpp::Duration::from_seconds(default_sleep_sec);
    } else if (now >= *sleep_until) {
      sleep_until.reset();
      current_index = (current_index + 1) % waypoints.size();
      auto next_wp = waypoints.at(current_index).pos;
      std::cout << "[TestPlanner] 次のポイントへ: " << next_wp.x() << ", " << next_wp.y()
                << std::endl;
    }
  }
  robot_commands.emplace_back(command->getMsg());

  return {PlannerBase::Status::RUNNING, robot_commands};
}

auto TestPlanner::getSelectedRobots(
  [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
  [[maybe_unused]] const std::unordered_map<uint8_t, RobotRole> & prev_roles)
  -> std::vector<uint8_t>
{
  if (ranges::count(selectable_robots, target_robot_id) > 0) {
    command =
      std::make_shared<crane::RobotCommandWrapper>("test_planner", target_robot_id, world_model);
    return {target_robot_id};
  } else {
    return {};
  }
}

auto TestPlanner::applyLegLimits(crane::RobotCommandWrapper & cmd, const Waypoint & wp) -> void
{
  double vmax = wp.max_velocity.has_value() ? *wp.max_velocity : default_max_velocity;
  double amax = wp.max_acceleration.has_value() ? *wp.max_acceleration : default_max_acceleration;
  cmd.setMaxVelocity("TestPlanner::applyLegLimits", vmax)
    .setMaxAcceleration("TestPlanner::applyLegLimits", amax);
}

auto TestPlanner::loadConfigFromFile(const std::string & path) -> bool
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (!root || !root.IsMap()) {
      RCLCPP_ERROR(rclcpp::get_logger("TestPlanner"), "設定ファイルのルートが不正です");
      return false;
    }

    if (root["robot_id"]) {
      target_robot_id = root["robot_id"].as<uint8_t>();
    }

    if (root["defaults"]) {
      const auto & d = root["defaults"];
      if (d["max_velocity"]) default_max_velocity = d["max_velocity"].as<double>();
      if (d["max_acceleration"]) default_max_acceleration = d["max_acceleration"].as<double>();
      if (d["sleep_sec"]) default_sleep_sec = d["sleep_sec"].as<double>();
    }

    // 経由点
    waypoints.clear();
    const auto & waypoints_node = root["waypoints"];
    if (!waypoints_node || !waypoints_node.IsSequence() || waypoints_node.size() == 0) {
      RCLCPP_WARN(rclcpp::get_logger("TestPlanner"), "設定に経由点がありません");
    } else {
      for (const auto & w : waypoints_node) {
        Waypoint wp;
        if (w["position"].IsSequence() && w["position"].size() >= 2) {
          wp.pos.x() = w["position"][0].as<double>();
          wp.pos.y() = w["position"][1].as<double>();
          // 3要素目がある場合は角度（度）として読み込み、ラジアンに変換
          if (w["position"].size() >= 3) {
            double theta_deg = w["position"][2].as<double>();
            wp.theta = theta_deg * M_PI / 180.0;
          }
        } else {
          RCLCPP_WARN(rclcpp::get_logger("TestPlanner"), "position が無い経由点をスキップしました");
          continue;
        }
        if (w["dwell_sec"]) wp.dwell_sec = w["dwell_sec"].as<double>();
        if (w["max_velocity"]) wp.max_velocity = w["max_velocity"].as<double>();
        if (w["max_acceleration"]) wp.max_acceleration = w["max_acceleration"].as<double>();
        waypoints.push_back(wp);
      }
    }

    return true;
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("TestPlanner"), "YAML エラー: %s", e.what());
    return false;
  }
}
}  // namespace crane
