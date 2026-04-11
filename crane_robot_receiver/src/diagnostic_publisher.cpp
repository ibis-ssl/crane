// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <fmt/format.h>

#include <cmath>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ping_status_array.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_robot_receiver/diagnostic_publisher.hpp>
#include <crane_visualization_interfaces/crane_visualizer_wrapper.hpp>
#include <cstring>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <map>
#include <range/v3/algorithm/contains.hpp>
#include <range/v3/algorithm/find_if.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
auto RobotData::updateErrorMap(
  const std::string & error_type, const std::string & message, int level,
  const rclcpp::Time & timestamp) -> bool
{
  // エラーが既存または変更された場合のみ更新
  bool is_new = !error_map.contains(error_type) || (error_map[error_type].level != level) ||
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
  if (error_map.contains(error_type)) {
    error_map.erase(error_type);
    has_error_changed = true;
    return true;
  }
  return false;
}

auto RobotData::isRobotDetected(const WorldModelWrapper & world_model, bool require_feedback) const
  -> bool
{
  for (const auto & robot_info : world_model.getMsg().robot_info_ours) {
    if (robot_info.id == robot_id) {
      return require_feedback ? (robot_info.available_vision || robot_info.available_feedback ||
                                 robot_info.available_tracker)
                              : (robot_info.available_vision || robot_info.available_tracker);
    }
  }
  return false;
}

auto RobotData::initializeDiagnostics(
  rclcpp::Node * node, WorldModelWrapper * world_model, bool sim_mode,
  crane_msgs::msg::PingStatusArray * latest_ping_msg,
  crane_msgs::msg::RobotFeedbackArray * latest_feedback_msg) -> void
{
  updater = std::make_unique<diagnostic_updater::Updater>(node);
  updater->setHardwareID(fmt::format("robot_{:02d}", robot_id));

  // 診断名のプレフィックス（aggregatorでのグループ化用）
  std::string diagnostic_prefix = fmt::format("robot_{:02d}/", robot_id);

  // 通信状態の診断
  updater->add(
    diagnostic_prefix + "communication",
    [this, node, world_model, sim_mode, latest_ping_msg,
     latest_feedback_msg](diagnostic_updater::DiagnosticStatusWrapper & stat) {
      if (!isRobotDetected(*world_model)) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot not detected");
        removeError("communication");
        return;
      }
      communicationDiagnosticCallback(
        stat, *latest_ping_msg, *latest_feedback_msg, node->now(), sim_mode);
    });

  // バッテリー状態の診断
  // 【循環参照回避】available_vision/feedback/trackerは診断結果に依存しないため安全
  updater->add(
    diagnostic_prefix + "battery", [this, node, world_model, sim_mode, latest_feedback_msg](
                                     diagnostic_updater::DiagnosticStatusWrapper & stat) {
      if (!isRobotDetected(*world_model, true)) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot not detected");
        removeError("battery");
        return;
      }
      batteryDiagnosticCallback(stat, *latest_feedback_msg, node->now(), sim_mode);
    });

  // ロボットエラーの診断
  // 【循環参照回避】available_vision/feedback/trackerは診断結果に依存しないため安全
  updater->add(
    diagnostic_prefix + "robot_error", [this, node, world_model, sim_mode, latest_feedback_msg](
                                         diagnostic_updater::DiagnosticStatusWrapper & stat) {
      if (!isRobotDetected(*world_model, true)) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Robot not detected");
        removeError("robot_error");
        return;
      }
      robotErrorDiagnosticCallback(stat, *latest_feedback_msg, node->now(), sim_mode);
    });

  // 診断情報の直接パブリッシャーを作成（後方互換性のため維持）
  direct_publisher = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    fmt::format("/diagnostics/robot_{:02d}", robot_id), 10);
}

auto RobotData::communicationDiagnosticCallback(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const crane_msgs::msg::PingStatusArray & ping_msg,
  const crane_msgs::msg::RobotFeedbackArray & feedback_msg, const rclcpp::Time & now_time,
  bool sim_mode) -> void
{
  auto ping = ranges::find_if(ping_msg.ping, [this](const crane_msgs::msg::PingStatus & msg) {
    return msg.robot_id == robot_id;
  });
  auto feedback = ranges::find_if(
    feedback_msg.feedback,
    [this](const crane_msgs::msg::RobotFeedback & msg) { return msg.robot_id == robot_id; });

  constexpr double FEEDBACK_WARN_AGE_MS = 100.0;
  constexpr double FEEDBACK_ERROR_AGE_MS = 250.0;
  constexpr double FEEDBACK_WARN_RATE_HZ = 20.0;
  constexpr double FEEDBACK_ERROR_RATE_HZ = 5.0;

  if (feedback != feedback_msg.feedback.end()) {
    std::string message = "Robot feedback healthy";
    int level = diagnostic_msgs::msg::DiagnosticStatus::OK;

    if (
      feedback->feedback_age_ms > FEEDBACK_ERROR_AGE_MS ||
      feedback->packet_frequency_hz < FEEDBACK_ERROR_RATE_HZ) {
      message = "Robot feedback degraded";
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (
      feedback->feedback_age_ms > FEEDBACK_WARN_AGE_MS ||
      feedback->packet_frequency_hz < FEEDBACK_WARN_RATE_HZ) {
      message = "Robot feedback warning";
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    stat.summary(level, message);
    stat.add("feedback_age_ms", feedback->feedback_age_ms);
    stat.add("feedback_packet_frequency_hz", feedback->packet_frequency_hz);
    stat.add("feedback_valid_packet_count", feedback->valid_packet_count);
    stat.add("feedback_invalid_packet_count", feedback->invalid_packet_count);
    stat.add("feedback_sync_error_count", feedback->sync_error_count);
    stat.add("feedback_checksum_error_count", feedback->checksum_error_count);
    stat.add("feedback_size_mismatch_count", feedback->size_mismatch_count);
    stat.add("feedback_counter_jump_count", feedback->counter_jump_count);
    if (ping != ping_msg.ping.end()) {
      stat.add("ping_ms", ping->ping_ms);
    }

    if (level > 0) {
      updateErrorMap("communication", message, level, now_time);
    } else {
      removeError("communication");
    }
    return;
  }

  if (ping != ping_msg.ping.end()) {
    std::string message = "Ping available, robot feedback missing";
    int level = diagnostic_msgs::msg::DiagnosticStatus::WARN;

    stat.summary(level, message);
    stat.add("ping_ms", ping->ping_ms);

    // エラーマップに登録または更新
    if (level > 0) {
      updateErrorMap("communication", message, level, now_time);
    } else {
      removeError("communication");
    }
  } else {
    // シミュレータ環境ではpingデータなしは正常
    if (sim_mode) {
      std::string message = "Simulation mode (no ping data)";
      int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.summary(level, message);
      removeError("communication");
    } else {
      // 実機環境ではERROR
      std::string message = "No robot telemetry received";
      int level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      stat.summary(level, message);
      updateErrorMap("communication", message, level, now_time);
    }
  }
}

auto RobotData::batteryDiagnosticCallback(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const crane_msgs::msg::RobotFeedbackArray & feedback_msg, const rclcpp::Time & now_time,
  bool sim_mode) -> void
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
    // シミュレータ環境ではfeedbackデータなしは正常
    if (sim_mode) {
      std::string message = "Simulation mode (no battery data)";
      int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      stat.summary(level, message);
      removeError("battery");
    } else {
      // 実機環境ではWARN（フィードバックなしでもVision/Trackerで制御可能）
      std::string message = "No robot feedback received";
      int level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      stat.summary(level, message);
      removeError("battery");
    }
  }
}

auto RobotData::robotErrorDiagnosticCallback(
  diagnostic_updater::DiagnosticStatusWrapper & stat,
  const crane_msgs::msg::RobotFeedbackArray & feedback_msg, const rclcpp::Time & now_time,
  bool sim_mode) -> void
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
    // シミュレータ環境ではfeedbackデータなしは正常
    if (sim_mode) {
      std::string message = "Simulation mode (no robot error data)";
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, message);
      removeError("robot_error");
    } else {
      // 実機環境ではWARN（フィードバックなしでもVision/Trackerで制御可能）
      std::string message = "No robot feedback received";
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
      removeError("robot_error");
    }
  }
}

DiagnosticPublisherNode::DiagnosticPublisherNode() : Node("diagnostic_publisher_node")
{
  declare_parameter("sim_mode", true);
  sim_mode_ = get_parameter("sim_mode").as_bool();

  declare_parameter("max_robot_id", 12);  // サポートする最大ロボットID
  int max_robot_id = get_parameter("max_robot_id").as_int();

  // 可視化用のCraneVisualizerBufferを初期化
  CraneVisualizerBuffer::activate(*this);
  visualizer_error = std::make_shared<VisualizerMessageBuilder>("receiver/diagnostic");

  // WorldModelの設定（ロボットデータ初期化より先に必要）
  world_model = std::make_unique<WorldModelWrapper>(*this);
  world_model->addCallback([this]() { worldModelCallback(); });

  // ロボットデータの初期化
  initializeRobots(max_robot_id);

  // サブスクリプションの設定
  ping_subscription = create_subscription<crane_msgs::msg::PingStatusArray>(
    "/ping", 10,
    [this](const crane_msgs::msg::PingStatusArray & msg) { pingMessageCallback(msg); });

  feedback_subscription = create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 10,
    [this](const crane_msgs::msg::RobotFeedbackArray & msg) { feedbackMessageCallback(msg); });

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
    robots_data.back()->initializeDiagnostics(
      this, world_model.get(), sim_mode_, &latest_ping_msg, &latest_feedback_msg);

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
  auto available_robot_ids = world_model->ours().robotsWhere().available().getIds();

  // ロボットの位置情報を更新
  for (const auto & robot : world_model->ours().robotsWhere().available().get()) {
    robot_positions[robot->id] = {
      robot->pose.pos.x(), robot->pose.pos.y(), robot->pose.theta, true};
  }

  // ロボットの状態を更新
  for (size_t id = 0; id < robots_data.size(); ++id) {
    auto & data = robots_data.at(id);
    // 状態を更新
    data->state = ranges::contains(available_robot_ids, static_cast<int>(id))
                    ? RobotState::ACTIVE
                    : RobotState::INACTIVE;
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
      if (robot_positions.contains(robot_id) && robot_positions[robot_id].valid) {
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

        visualizer_error->drawCenteredLabel(
          Point(
            robot_positions[robot_id].x,
            robot_positions[robot_id].y + text_offset + ERROR_TEXT_OFFSET),
          error_info.message, color, 50.0);

        text_offset += ERROR_TEXT_INCREMENT;
      }
    }
  }

  // 可視化情報を送信
  visualizer_error->flush();
  CraneVisualizerBuffer::publish();
}

}  // namespace crane
