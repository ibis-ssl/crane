// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_RECEIVER__DIAGNOSTIC_PUBLISHER_HPP_
#define CRANE_ROBOT_RECEIVER__DIAGNOSTIC_PUBLISHER_HPP_

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_robot_receiver/robot_errors.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
// ロボット位置情報の構造体
struct RobotPosition
{
  double x;
  double y;
  double theta;
  bool valid;
};

// ロボットごとのデータ管理クラス
class RobotData
{
public:
  explicit RobotData(const uint8_t & id) : robot_id(id) {}

  // ロボットの診断情報の初期化
  auto initializeDiagnostics(rclcpp::Node * node) -> void;

  // エラーマップの更新関数
  auto updateErrorMap(
    const std::string & error_type, const std::string & message, int level,
    const rclcpp::Time & timestamp) -> bool;

  // エラーマップからエラーを削除
  auto removeError(const std::string & error_type) -> bool;

  uint8_t robot_id;

  RobotState state = RobotState::INACTIVE;

  rclcpp::Time last_update_time;

  std::unique_ptr<diagnostic_updater::Updater> updater;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr direct_publisher;

  // エラー情報を保存するマップ (エラータイプ => エラー情報)
  std::map<std::string, ErrorInfo> error_map;

  // 前回のエラー状態と比較して変化があるかをチェックするフラグ
  bool has_error_changed = false;

private:
  // 診断情報更新コールバック関数
  auto communicationDiagnosticCallback(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    const crane_msgs::msg::PingStatusArray & ping_msg, const rclcpp::Time & now_time) -> void;

  auto batteryDiagnosticCallback(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    const crane_msgs::msg::RobotFeedbackArray & feedback_msg, const rclcpp::Time & now_time)
    -> void;

  auto robotErrorDiagnosticCallback(
    diagnostic_updater::DiagnosticStatusWrapper & stat,
    const crane_msgs::msg::RobotFeedbackArray & feedback_msg, const rclcpp::Time & now_time)
    -> void;
};

class DiagnosticPublisherNode : public rclcpp::Node
{
public:
  DiagnosticPublisherNode();

private:
  // ロボットの診断情報の初期化
  auto initializeRobots(int max_robot_id) -> void;

  // ロボットエラーの可視化を行う関数
  auto visualizeRobotErrors() -> void;

  // メッセージコールバック
  auto pingMessageCallback(const crane_msgs::msg::PingStatusArray & msg) -> void;
  auto feedbackMessageCallback(const crane_msgs::msg::RobotFeedbackArray & msg) -> void;
  auto worldModelCallback() -> void;

  WorldModelWrapper::UniquePtr world_model;
  std::vector<std::shared_ptr<RobotData>> robots_data;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::TimerBase::SharedPtr visualization_timer;  // 可視化用のタイマー

  rclcpp::Subscription<crane_msgs::msg::PingStatusArray>::SharedPtr ping_subscription;
  crane_msgs::msg::PingStatusArray latest_ping_msg;

  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr feedback_subscription;
  crane_msgs::msg::RobotFeedbackArray latest_feedback_msg;

  // 可視化ラッパー
  std::shared_ptr<VisualizerMessageBuilder> visualizer_error;

  // エラーが発生したロボットの位置情報を保持
  std::map<uint8_t, RobotPosition> robot_positions;
};
}  // namespace crane
#endif  // CRANE_ROBOT_RECEIVER__DIAGNOSTIC_PUBLISHER_HPP_
