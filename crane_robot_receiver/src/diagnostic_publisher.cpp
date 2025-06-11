// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cmath>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_robot_receiver/diagnostic_publisher.hpp>
#include <cstring>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
// RobotDataクラスのメソッド実装
auto RobotData::updateErrorMap(
  const std::string & error_type, const std::string & message, int level,
  const rclcpp::Time & timestamp) -> bool
{
  // エラーが既存または変更された場合のみ更新
  bool is_new = (error_map.count(error_type) == 0) || (error_map[error_type].level != level) ||
                (error_map[error_type].message != message);

  if (is_new) {
    error_map[error_type] = {
      error_type,  // タイプ
      message,     // メッセージ
      level,       // レベル
      timestamp    // タイムスタンプ
    };
    has_error_changed = true;
    return true;
  }
  return false;
}

auto RobotData::removeError(const std::string & error_type) -> bool
{
  if (error_map.count(error_type) > 0) {
    error_map.erase(error_type);
    has_error_changed = true;
    return true;
  }
  return false;
}

auto RobotData::initializeDiagnostics(rclcpp::Node * node) -> void
{
  updater = std::make_unique<diagnostic_updater::Updater>(node);
  updater->setHardwareID("robot_" + std::to_string(robot_id));

  auto ping_msg_ptr = std::make_shared<crane_msgs::msg::PingStatusArray>();
  auto feedback_msg_ptr = std::make_shared<crane_msgs::msg::RobotFeedbackArray>();

  // 通信状態の診断
  updater->add(
    "communication",
    [this, ping_msg_ptr, node](diagnostic_updater::DiagnosticStatusWrapper & stat) {
      if (state != RobotState::ACTIVE) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
        removeError("communication");
        return;
      }
      communicationDiagnosticCallback(stat, *ping_msg_ptr, node->now());
    });

  // バッテリー状態の診断
  updater->add(
    "battery", [this, feedback_msg_ptr, node](diagnostic_updater::DiagnosticStatusWrapper & stat) {
      if (state != RobotState::ACTIVE) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
        removeError("battery");
        return;
      }
      batteryDiagnosticCallback(stat, *feedback_msg_ptr, node->now());
    });

  // ロボットエラーの診断
  updater->add(
    "robot_error",
    [this, feedback_msg_ptr, node](diagnostic_updater::DiagnosticStatusWrapper & stat) {
      if (state != RobotState::ACTIVE) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot inactive");
        removeError("robot_error");
        return;
      }
      robotErrorDiagnosticCallback(stat, *feedback_msg_ptr, node->now());
    });

  // 診断情報の直接パブリッシャーを作成
  direct_publisher = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics/robot_" + std::to_string(robot_id), 10);
}

auto RobotData::communicationDiagnosticCallback(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const crane_msgs::msg::PingStatusArray & ping_msg, const rclcpp::Time & now_time) -> void
{
  auto ping = ranges::find_if(ping_msg.ping, [this](const crane_msgs::msg::PingStatus & msg) {
    return msg.robot_id == robot_id;
  });

  if (ping != ping_msg.ping.end()) {
    std::string message;
    int level;

    if (ping->ping_ms > 50.0) {
      message = "Ping time high";
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (ping->ping_ms > 10.0) {
      message = "Communication latency medium";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      message = "Communication OK";
      level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }

    stat.summary(level, message);
    stat.add("ping_ms", ping->ping_ms);

    // エラーマップに登録または更新
    if (level > 0) {
      updateErrorMap("communication", message, level, now_time);
    } else {
      removeError("communication");
    }
  } else {
    std::string message = "No ping data received";
    int level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    stat.summary(level, message);
    updateErrorMap("communication", message, level, now_time);
  }
}

auto RobotData::batteryDiagnosticCallback(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const crane_msgs::msg::RobotFeedbackArray & feedback_msg, const rclcpp::Time & now_time) -> void
{
  auto feedback = ranges::find_if(
    feedback_msg.feedback,
    [this](const crane_msgs::msg::RobotFeedback & msg) { return msg.robot_id == robot_id; });

  if (feedback != feedback_msg.feedback.end()) {
    std::string message;
    int level;

    if (feedback->voltage[0] < 22.0) {
      message = "Low battery voltage";
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (feedback->voltage[0] < 23.0) {
      message = "Battery voltage medium";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      message = "Battery voltage high";
      level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }

    stat.summary(level, message);
    stat.add("voltage", feedback->voltage[0]);

    // エラーマップに登録または更新
    if (level > 0) {
      updateErrorMap("battery", message, level, now_time);
    } else {
      removeError("battery");
    }
  } else {
    std::string message = "No robot feedback received";
    int level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

    stat.summary(level, message);
    updateErrorMap("battery", message, level, now_time);
  }
}

auto RobotData::robotErrorDiagnosticCallback(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const crane_msgs::msg::RobotFeedbackArray & feedback_msg, const rclcpp::Time & now_time) -> void
{
  auto feedback = ranges::find_if(
    feedback_msg.feedback,
    [this](const crane_msgs::msg::RobotFeedback & msg) { return msg.robot_id == robot_id; });

  if (feedback != feedback_msg.feedback.end()) {
    if (feedback->error_id != 0 || feedback->error_info != 0) {
      // エラー情報を人間が読める形式に変換して表示
      std::string error_str =
        utils::convertErrorDataToStr(feedback->error_id, feedback->error_info);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_str);

      stat.add("error_id", feedback->error_id);
      stat.add("error_info", feedback->error_info);
      stat.add("error_value", feedback->error_value);
      stat.add("error_description", error_str);

      // エラーマップに登録
      updateErrorMap(
        "robot_error", error_str, diagnostic_msgs::msg::DiagnosticStatus::ERROR, now_time);
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No error");
      removeError("robot_error");
    }
  } else {
    std::string message = "No robot feedback received";
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
    updateErrorMap("robot_error", message, diagnostic_msgs::msg::DiagnosticStatus::ERROR, now_time);
  }
}

DiagnosticPublisherNode::DiagnosticPublisherNode() : Node("diagnostic_publisher_node")
{
  declare_parameter("max_robot_id", 12);  // サポートする最大ロボットID
  int max_robot_id = get_parameter("max_robot_id").as_int();

  // 可視化用のCraneVisualizerBufferを初期化
  CraneVisualizerBuffer::activate(*this);
  visualizer_error = std::make_shared<VisualizerMessageBuilder>("diagnostic_errors");

  // ロボットデータの初期化
  initializeRobots(max_robot_id);

  // サブスクリプションの設定
  ping_subscription = create_subscription<crane_msgs::msg::PingStatusArray>(
    "/ping", 10,
    [this](const crane_msgs::msg::PingStatusArray & msg) { pingMessageCallback(msg); });

  feedback_subscription = create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 10,
    [this](const crane_msgs::msg::RobotFeedbackArray & msg) { feedbackMessageCallback(msg); });

  // WorldModelの設定
  world_model = std::make_unique<WorldModelWrapper>(*this);
  world_model->addCallback([this]() { worldModelCallback(); });

  // 診断情報更新タイマー
  timer = this->create_wall_timer(std::chrono::seconds(1), [this]() {
    for (auto & robot_data : robots_data) {
      if (robot_data->updater) {
        robot_data->updater->force_update();
        robot_data->last_update_time = now();
      }
    }
  });

  // 可視化用タイマー - より高頻度で更新
  visualization_timer =
    this->create_wall_timer(std::chrono::milliseconds(100), [this]() { visualizeRobotErrors(); });
}

// ロボットデータの初期化
auto DiagnosticPublisherNode::initializeRobots(int max_robot_id) -> void
{
  // 最大ロボット数分の構造体を事前に準備
  for (int i = 0; i <= max_robot_id; ++i) {
    // 初期状態では空のRobotDataを作成
    robots_data.emplace_back(std::make_shared<RobotData>(i));
    robots_data.back()->initializeDiagnostics(this);

    // ロボット位置の初期化
    robot_positions[i] = {0.0, 0.0, 0.0, false};
  }
}

auto DiagnosticPublisherNode::pingMessageCallback(const crane_msgs::msg::PingStatusArray & msg)
  -> void
{
  latest_ping_msg = msg;
}

auto DiagnosticPublisherNode::feedbackMessageCallback(
  const crane_msgs::msg::RobotFeedbackArray & msg) -> void
{
  latest_feedback_msg = msg;
}

auto DiagnosticPublisherNode::worldModelCallback() -> void
{
  auto available_robot_ids = world_model->ours().getAvailableRobotIds();

  // ロボットの位置情報を更新
  for (const auto & robot : world_model->ours().getAvailableRobots()) {
    robot_positions[robot->id] = {
      robot->pose.pos.x(), robot->pose.pos.y(), robot->pose.theta, true};
  }

  // ロボットの状態を更新
  for (int id = 0; id < robots_data.size(); ++id) {
    auto & data = robots_data.at(id);
    // 状態を更新
    data->state =
      ranges::contains(available_robot_ids, id) ? RobotState::ACTIVE : RobotState::INACTIVE;
  }
}

// ロボットエラーの可視化を行う関数
auto DiagnosticPublisherNode::visualizeRobotErrors() -> void
{
  visualizer_error->clear();

  // 現在のタイムスタンプ
  rclcpp::Time now = this->now();
  constexpr double ERROR_DISPLAY_TIMEOUT = 10.0;  // 10秒以上経過したエラーは表示しない
  constexpr double ERROR_MARKER_RADIUS = 0.15;    // ロボット周囲の円の半径
  constexpr double ERROR_TEXT_OFFSET = 0.15;      // エラーテキストの初期オフセット
  constexpr double ERROR_TEXT_INCREMENT = 0.08;   // 複数エラー時の増分

  // 各ロボットのエラー情報を可視化
  for (const auto & robot_data : robots_data) {
    if (robot_data->state != RobotState::ACTIVE) {
      continue;  // 非アクティブなロボットはスキップ
    }

    // ロボットのエラー情報を処理
    double text_offset = 0.0;

    for (const auto & [error_type, error_info] : robot_data->error_map) {
      // エラーと警告のみを表示 (0: OK, 1: WARN, 2: ERROR, 3: STALE)
      if (error_info.level <= 0) {
        continue;
      }

      // 古いエラーは表示しない
      if ((now - error_info.timestamp).seconds() > ERROR_DISPLAY_TIMEOUT) {
        continue;
      }

      // ロボットIDを取得
      uint8_t robot_id = robot_data->robot_id;

      // ロボット位置が有効な場合のみ表示
      if (robot_positions.count(robot_id) > 0 && robot_positions[robot_id].valid) {
        // 最初のエラーでマーカー円を表示
        if (text_offset == 0.0) {
          // エラーレベルに応じた色を設定
          std::string color = utils::getColorForErrorLevel(error_info.level);
          constexpr double opacity = 0.8;

          // ロボット周囲に円形マーカーを表示
          visualizer_error->circle()
            .center(robot_positions[robot_id].x, robot_positions[robot_id].y)
            .radius(ERROR_MARKER_RADIUS)
            .stroke(color, opacity)
            .strokeWidth(2.0)
            .fill("none")
            .build();
        }

        // エラーメッセージを表示
        std::string color = utils::getColorForErrorLevel(error_info.level);
        constexpr double opacity = 0.8;

        visualizer_error->text()
          .position(
            robot_positions[robot_id].x,
            robot_positions[robot_id].y + text_offset + ERROR_TEXT_OFFSET)
          .text(error_info.message)
          .fill(color, opacity)
          .fontSize(50.0)
          .textAnchor("middle")
          .build();

        text_offset += ERROR_TEXT_INCREMENT;
      }
    }
  }

  // 可視化情報を送信
  visualizer_error->flush();
  CraneVisualizerBuffer::publish();
}

}  // namespace crane
