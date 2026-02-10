// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crane_sessions/test_session.hpp>
#include <filesystem>
#include <range/v3/algorithm/count.hpp>

namespace crane
{
TestSession::TestSession(WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
: SessionBase("test", world_model), topics_interface(node.get_node_topics_interface())
{
  config_file_path =
    (std::filesystem::path(ament_index_cpp::get_package_share_directory("crane_sessions")) /
     "config" / "test_planner.yaml")
      .string();
  if (not loadConfigFromFile(config_file_path)) {
    RCLCPP_WARN(
      rclcpp::get_logger("TestSession"), "設定の読込に失敗: %s", config_file_path.c_str());
  }
  reload_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
    topics_interface, "/test_planner/reload", rclcpp::QoS(1),
    [this](std_msgs::msg::Empty::ConstSharedPtr) { reload_requested = true; });
}

std::pair<SessionBase::Status, std::vector<crane_msgs::msg::RobotCommand>>
TestSession::calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
{
  std::vector<crane_msgs::msg::RobotCommand> robot_commands;
  if (robots.empty()) {
    return {SessionBase::Status::RUNNING, robot_commands};
  }

  // リロード要求に対応
  if (reload_requested) {
    if (not loadConfigFromFile(config_file_path)) {
      RCLCPP_WARN(
        rclcpp::get_logger("TestSession"), "設定の読込に失敗: %s", config_file_path.c_str());
    }
    current_index = 0;
    sleep_until.reset();
    reload_requested = false;
  }

  // commandを初期化（まだ初期化されていない場合）
  auto robot_id = robots[0];
  if (!command) {
    command =
      std::make_shared<crane::PositionCommandWrapper>("test_tactic", robot_id.id, world_model);
  }

  auto now = rclcpp::Clock().now();

  // 待機時間経過後、次のウェイポイントに遷移
  if (sleep_until.has_value()) {
    double remaining_sec = (*sleep_until - now).seconds();
    if (remaining_sec > 0.1) {  // 0.1秒以上残っている場合のみログ出力（頻度削減）
      static rclcpp::Time last_log_time;
      if ((now - last_log_time).seconds() > 1.0) {  // 1秒ごとにログ
        RCLCPP_INFO(
          rclcpp::get_logger("TestSession"), "Sleeping... remaining: %.1f sec", remaining_sec);
        last_log_time = now;
      }
    }

    if (now >= *sleep_until) {
      sleep_until.reset();
      current_index = (current_index + 1) % waypoints.size();
      RCLCPP_INFO(
        rclcpp::get_logger("TestSession"), "Next waypoint[%zu]: %.2f, %.2f", current_index,
        waypoints.at(current_index).pos.x(), waypoints.at(current_index).pos.y());
    }
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

  // 目標位置に到達したら待機開始
  if (command->getTargetDistance() < 0.05 && not sleep_until.has_value()) {
    RCLCPP_INFO(
      rclcpp::get_logger("TestSession"), "Arrived at waypoint[%zu], sleeping for %.1f sec",
      current_index, default_sleep_sec);
    sleep_until = now + rclcpp::Duration::from_seconds(default_sleep_sec);
  }
  robot_commands.emplace_back(command->getMsg());

  return {SessionBase::Status::RUNNING, robot_commands};
}

auto TestSession::applyLegLimits(crane::PositionCommandWrapper & cmd, const Waypoint & wp) -> void
{
  double vmax = wp.max_velocity.has_value() ? *wp.max_velocity : default_max_velocity;
  double amax = wp.max_acceleration.has_value() ? *wp.max_acceleration : default_max_acceleration;
  cmd.setMaxVelocity("TestSession::applyLegLimits", vmax)
    .setMaxAcceleration("TestSession::applyLegLimits", amax);
}

auto TestSession::loadConfigFromFile(const std::string & path) -> bool
{
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (!root || !root.IsMap()) {
      RCLCPP_ERROR(rclcpp::get_logger("TestSession"), "設定ファイルのルートが不正です");
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
    RCLCPP_INFO(
      rclcpp::get_logger("TestSession"), "設定読込完了: sleep_sec=%.1f, max_vel=%.1f, max_acc=%.1f",
      default_sleep_sec, default_max_velocity, default_max_acceleration);

    // 経由点
    waypoints.clear();
    const auto & waypoints_node = root["waypoints"];
    if (!waypoints_node || !waypoints_node.IsSequence() || waypoints_node.size() == 0) {
      RCLCPP_WARN(rclcpp::get_logger("TestSession"), "設定に経由点がありません");
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
          RCLCPP_WARN(rclcpp::get_logger("TestSession"), "position が無い経由点をスキップしました");
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
    RCLCPP_ERROR(rclcpp::get_logger("TestSession"), "YAML エラー: %s", e.what());
    return false;
  }
}
}  // namespace crane
