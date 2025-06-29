// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/world_model_data_provider.hpp"

#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <crane_msg_wrappers/delay_monitor_wrapper.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <robocup_ssl_msgs/msg/robot_id.hpp>
#include <string>
#include <vector>

namespace crane
{

WorldModelDataProvider::WorldModelDataProvider(rclcpp::Node & node)
: node(node), vis_data_handler(node)
{
  using std::chrono_literals::operator""ms;

  vision_processor_ = std::make_unique<VisionDataProcessor>(node);
  tracker_processor_ = std::make_unique<TrackerDataProcessor>(node);

  area_mask.min_corner() << -20., -10.;
  area_mask.max_corner() << 20., 10.;

  tracker_processor_->setAreaMask(area_mask);

  vision_processor_->setVisualizationHandler(
    [this](const SSL_GeometryData & geometry_data, bool half_court_mode) {
      vis_data_handler.flushGeometryVisualization(geometry_data, half_court_mode);
    });

  vision_processor_->setGeometryUpdateHandler([this]() {
    // Update geometry data whenever vision geometry is received
    updateGeometryIfNeeded();
  });

  udp_timer = node.create_wall_timer(10ms, std::bind(&WorldModelDataProvider::on_udp_timer, this));

  for (int i = 0; i < 20; i++) {
    crane_msgs::msg::RobotInfo info;
    info.vision_detected = false;
    info.feedback_detected = false;
    info.detected = false;
    info.id = i;
    data.robot_info[0].emplace_back(info);
    data.robot_info[1].emplace_back(info);
  }

  // /play_situationのトピック統計はsession_controllerで取得
  sub_play_situation = node.create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 1,
    [this](const crane_msgs::msg::PlaySituation msg) { latest_play_situation = msg; });

  sub_robot_feedback = node.create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 1, [this](const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg) {
      robot_feedback = *msg;
      auto now = rclcpp::Clock().now();

      // VisionDataProcessorへのフィードバック統合（味方ロボットのみ）
      for (const auto & feedback : msg->feedback) {
        if (feedback.robot_id < 20) {  // ID範囲チェック
          vision_processor_->updateFriendlyRobotFeedback(feedback.robot_id, feedback);
        }
      }

      // 従来の処理（互換性維持）
      for (auto & robot : data.robot_info[0]) {
        auto & contact = robot.ball_contact;
        contact.current_time = now;
        if (auto feedback = std::find_if(
              robot_feedback.feedback.begin(), robot_feedback.feedback.end(),
              [&](const crane_msgs::msg::RobotFeedback & f) { return f.robot_id == robot.id; });
            feedback != robot_feedback.feedback.end()) {
          contact.is_vision_source = false;
          if (feedback->ball_sensor) {
            contact.last_contacted_time = now;
          }
          // 範囲内参照で実行時エラー
          // data.ball_sensor_detected[robot.id] = feedback->ball_sensor;
          auto & robot_info = data.robot_info[static_cast<uint8_t>(game_data.our_color)][robot.id];
          robot_info.ball_sensor = feedback->ball_sensor;
          robot_info.last_ball_sensor_stamp = now;
          robot_info.feedback_detected = true;
          robot_info.last_feedback_detection_stamp = now;
          if (not robot_info.vision_detected) {
            try {
              // odom_speedはグローバル座標系
              robot_info.velocity.x = feedback->odom_speed[0];
              robot_info.velocity.y = feedback->odom_speed[1];
              robot_info.velocity_norm = std::hypot(robot_info.velocity.x, robot_info.velocity.y);
              // feedbackは100Hz
              // robot_info.pose.x += robot_info.velocity.x * 0.01;
              // robot_info.pose.y += robot_info.velocity.y * 0.01;
              robot_info.pose.x = feedback->odom[0];
              robot_info.pose.y = feedback->odom[1];
              // yaw_angleはdeg
              using boost::math::constants::degree;
              robot_info.pose.theta = feedback->yaw_angle * degree<double>();
            } catch (...) {
              std::cout << "feedback->odom_speed has noe element" << std::endl;
            }
          }
        } else {
          try {
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][robot.id].feedback_detected =
              false;
          } catch (...) {
            std::cout << "aaaaaaaaaa element" << std::endl;
          }
        }
      }
    });

  sub_robots_status_blue = node.create_subscription<robocup_ssl_msgs::msg::RobotsStatus>(
    "/robots_status/blue", 1, [this](const robocup_ssl_msgs::msg::RobotsStatus::SharedPtr msg) {
      if (game_data.our_color == Color::BLUE) {
        auto now = rclcpp::Clock().now();
        for (auto status : msg->robots_status) {
          // data.ball_sensor_detected[status.robot_id] = status.infrared;
          auto & robot =
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][status.robot_id];
          robot.ball_sensor = status.infrared;
          robot.last_ball_sensor_stamp = now;
          auto & contact =
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][status.robot_id]
              .ball_contact;
          contact.current_time = now;
          contact.is_vision_source = false;
          if (status.infrared) {
            contact.last_contacted_time = now;
          }
        }
      }
    });

  sub_robots_status_yellow = node.create_subscription<robocup_ssl_msgs::msg::RobotsStatus>(
    "/robots_status/yellow", 1, [this](const robocup_ssl_msgs::msg::RobotsStatus::SharedPtr msg) {
      if (game_data.our_color == Color::YELLOW) {
        auto now = rclcpp::Clock().now();
        for (auto status : msg->robots_status) {
          // data.ball_sensor_detected[status.robot_id] = status.infrared;
          auto & robot =
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][status.robot_id];
          robot.ball_sensor = status.infrared;
          robot.last_ball_sensor_stamp = now;

          auto & contact =
            data.robot_info[static_cast<uint8_t>(game_data.our_color)][status.robot_id]
              .ball_contact;
          contact.current_time = now;
          contact.is_vision_source = false;
          if (status.infrared) {
            contact.last_contacted_time = now;
          }
        }
      }
    });

  node.declare_parameter("team_name", "ibis-ssl");
  game_data.team_name = node.get_parameter("team_name").as_string();

  node.declare_parameter("initial_team_color", "BLUE");
  auto initial_team_color = node.get_parameter("initial_team_color").as_string();
  if (initial_team_color == "BLUE") {
    game_data.our_color = Color::BLUE;
    game_data.their_color = Color::YELLOW;
  } else {
    game_data.our_color = Color::YELLOW;
    game_data.their_color = Color::BLUE;
  }

  node.declare_parameter("is_emplace_positive_side", true);
  is_emplace_positive_side = node.get_parameter("is_emplace_positive_side").get_value<bool>();

  sub_referee = node.create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 1, [this](const robocup_ssl_msgs::msg::Referee & msg) {
      if (msg.yellow.name == game_data.team_name) {
        // YELLOW
        game_data.our_color = Color::YELLOW;
        game_data.their_color = Color::BLUE;
        game_data.our_goalie_id = msg.yellow.goalkeeper;
        game_data.their_goalie_id = msg.blue.goalkeeper;
        if (not msg.yellow.max_allowed_bots.empty()) {
          game_data.our_max_allowed_bots = msg.yellow.max_allowed_bots[0];
        }
        if (not msg.blue.max_allowed_bots.empty()) {
          game_data.their_max_allowed_bots = msg.blue.max_allowed_bots[0];
        }
        if (not msg.blue_team_on_positive_half.empty()) {
          on_positive_half = not msg.blue_team_on_positive_half[0];
        }
      } else if (msg.blue.name == game_data.team_name) {
        // BLUE
        game_data.our_color = Color::BLUE;
        game_data.their_color = Color::YELLOW;
        game_data.our_goalie_id = msg.blue.goalkeeper;
        game_data.their_goalie_id = msg.yellow.goalkeeper;
        if (not msg.blue.max_allowed_bots.empty()) {
          game_data.our_max_allowed_bots = msg.blue.max_allowed_bots[0];
        }
        if (not msg.yellow.max_allowed_bots.empty()) {
          game_data.their_max_allowed_bots = msg.yellow.max_allowed_bots[0];
        }
        if (not msg.blue_team_on_positive_half.empty()) {
          on_positive_half = msg.blue_team_on_positive_half[0];
        }
      } else {
        std::stringstream what;
        what << "Cannot find our team name, " << std::string(game_data.team_name)
             << " in referee message. ";
        what << "blue team name: " << std::string(msg.blue.name)
             << ", yellow team name: " << std::string(msg.yellow.name);
        //        throw std::runtime_error(what.str());
      }

      if (not msg.designated_position.empty()) {
        data.ball_placement_target_x = msg.designated_position.front().x / 1000.;
        data.ball_placement_target_y = msg.designated_position.front().y / 1000.;
      }
      // チーム色をVisionDataProcessorに設定
      if (game_data.our_color == Color::BLUE) {
        vision_processor_->setOurTeamColor(VisionDataProcessor::Color::BLUE);
      } else {
        vision_processor_->setOurTeamColor(VisionDataProcessor::Color::YELLOW);
      }

      vis_data_handler.flushRefereeVisualization(msg, game_data.field_w, game_data.field_h);
      CraneVisualizerBuffer::publish();
    });

  // ロボットコマンド購読（味方ロボットの制御情報統合用）
  sub_robot_commands = node.create_subscription<crane_msgs::msg::RobotCommands>(
    "/robot_commands", 1, [this](const crane_msgs::msg::RobotCommands::SharedPtr msg) {
      for (const auto & command : msg->robot_commands) {
        if (command.robot_id < 20) {  // ID範囲チェック
          Eigen::Vector2d cmd_vel(command.current_velocity.x, command.current_velocity.y);
          double cmd_omega = command.current_velocity.theta;
          vision_processor_->updateFriendlyRobotCommand(command.robot_id, cmd_vel, cmd_omega);
        }
      }
    });
}

auto WorldModelDataProvider::on_udp_timer() -> void
{
  tracker_processor_->processTrackerPackets();
  vision_processor_->processVisionPackets();

  // Check if geometry needs to be updated after processing vision packets
  if (!geometry_initialized) {
    updateGeometryIfNeeded();
  }
}

auto WorldModelDataProvider::updateGeometryIfNeeded() -> void
{
  // Check if vision processor has valid geometry data (non-zero field dimensions)
  double field_w = vision_processor_->getFieldWidth();
  double field_h = vision_processor_->getFieldHeight();

  if (field_w <= 0.0 || field_h <= 0.0) {
    // Vision geometry not yet available, skip update
    return;
  }

  // Check if geometry has actually changed to avoid unnecessary updates
  bool geometry_changed = !geometry_initialized || std::abs(game_data.field_w - field_w) > 1e-6 ||
                          std::abs(game_data.field_h - field_h) > 1e-6;

  // Update geometry data from vision processor
  game_data.field_w = field_w;
  game_data.field_h = field_h;
  game_data.goal_w = vision_processor_->getGoalWidth();
  game_data.goal_h = vision_processor_->getGoalHeight();
  game_data.penalty_area_w = vision_processor_->getPenaltyAreaWidth();
  game_data.penalty_area_h = vision_processor_->getPenaltyAreaHeight();

  // Log geometry update for debugging
  if (geometry_changed) {
    RCLCPP_INFO(
      node.get_logger(),
      "Field geometry %s: field=%.3fx%.3f, goal=%.3fx%.3f, penalty_area=%.3fx%.3f",
      geometry_initialized ? "updated" : "initialized", game_data.field_w, game_data.field_h,
      game_data.goal_w, game_data.goal_h, game_data.penalty_area_w, game_data.penalty_area_h);
  }

  constexpr double OFFSET = 0.3;
  area_mask.min_corner() << -0.5 * game_data.field_w - OFFSET, -0.5 * game_data.field_h - OFFSET;
  area_mask.max_corner() << 0.5 * game_data.field_w + OFFSET, 0.5 * game_data.field_h + OFFSET;

  tracker_processor_->setAreaMask(area_mask);

  geometry_initialized = true;
}

crane_msgs::msg::WorldModel WorldModelDataProvider::getMsg()
{
  crane_msgs::msg::WorldModel msg;
  msg.is_yellow = (game_data.our_color == Color::YELLOW);
  msg.on_positive_half = on_positive_half;
  msg.is_emplace_positive_side = is_emplace_positive_side;
  msg.our_max_allowed_bots = game_data.our_max_allowed_bots;
  msg.their_max_allowed_bots = game_data.their_max_allowed_bots;

  // Get ball info from vision processor (primary) or tracker processor (fallback)
  if (vision_processor_->hasVisionUpdated()) {
    msg.ball_info = vision_processor_->getBallInfo();
  } else if (tracker_processor_->hasTrackerUpdated()) {
    msg.ball_info = tracker_processor_->getBallInfo();
  } else {
    // No data available, use last known state
    msg.ball_info = data.ball_info;
  }

  // Get robot info from vision processor (primary with EKF) or tracker processor (fallback)
  auto vision_robots_0 = vision_processor_->getRobotInfo(0);
  auto vision_robots_1 = vision_processor_->getRobotInfo(1);
  auto tracker_robots_0 = tracker_processor_->getRobotInfo(0);
  auto tracker_robots_1 = tracker_processor_->getRobotInfo(1);

  // データソース使用状況の定期ログ（5秒間隔）
  static rclcpp::Time last_source_log_time = rclcpp::Clock(RCL_ROS_TIME).now();
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();
  if ((now - last_source_log_time).seconds() > 5.0) {
    bool vision_available = vision_processor_->hasVisionUpdated();
    bool tracker_available = tracker_processor_->hasTrackerUpdated();
    RCLCPP_INFO(
      node.get_logger(), "Data source status: Vision=%s, Tracker=%s",
      vision_available ? "ACTIVE" : "INACTIVE", tracker_available ? "ACTIVE" : "INACTIVE");
    last_source_log_time = now;
  }

  // チーム0のロボット処理（VisionプライマリでEKFフィルタリング済み）
  for (size_t i = 0; i < vision_robots_0.size() && i < data.robot_info[0].size(); ++i) {
    auto merged_robot = vision_robots_0[i];

    // フィードバック情報をマージ（従来の処理から継承）
    merged_robot.feedback_detected = data.robot_info[0][i].feedback_detected;
    merged_robot.ball_sensor = data.robot_info[0][i].ball_sensor;
    merged_robot.last_ball_sensor_stamp = data.robot_info[0][i].last_ball_sensor_stamp;
    merged_robot.last_feedback_detection_stamp =
      data.robot_info[0][i].last_feedback_detection_stamp;
    merged_robot.detected = merged_robot.vision_detected or merged_robot.feedback_detected;

    // 外部トラッカーとVisionの比較ログ（両方利用可能な場合）
    if (
      merged_robot.vision_detected && i < tracker_robots_0.size() &&
      tracker_robots_0[i].vision_detected) {
      const auto & tracker_robot = tracker_robots_0[i];
      double vision_angle = merged_robot.pose.theta;
      double tracker_angle = tracker_robot.pose.theta;
      double angle_diff = std::abs(vision_angle - tracker_angle);
      // 角度差を[-π, π]範囲に正規化
      if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
      }

      // 角度差が大きい場合（10度以上）はログ出力
      if (angle_diff > M_PI / 18.0) {  // 10度 = π/18 ラジアン
        RCLCPP_WARN(
          node.get_logger(),
          "Team0 Robot %zu: Tracker vs Vision angle difference = %.2f deg (Tracker: %.2f, Vision: "
          "%.2f)",
          i, angle_diff * 180.0 / M_PI, tracker_angle * 180.0 / M_PI, vision_angle * 180.0 / M_PI);
      }
    }

    // Visionデータが無い場合はトラッカーからフォールバック
    if (!merged_robot.vision_detected && i < tracker_robots_0.size()) {
      const auto & tracker_robot = tracker_robots_0[i];
      if (tracker_robot.vision_detected) {
        RCLCPP_DEBUG(
          node.get_logger(), "Team0 Robot %zu: Using tracker fallback (angle: %.2f deg)", i,
          tracker_robot.pose.theta * 180.0 / M_PI);
        merged_robot.pose = tracker_robot.pose;
        merged_robot.velocity = tracker_robot.velocity;
        merged_robot.velocity_norm = tracker_robot.velocity_norm;
        merged_robot.vision_detected = true;
        merged_robot.detected = true;
      }
    }

    if (static_cast<uint8_t>(game_data.our_color) == 0) {
      if (ranges::contains(robot_ids_mask, merged_robot.id)) {
        msg.robot_info_theirs.emplace_back(merged_robot);
      } else {
        msg.robot_info_ours.emplace_back(merged_robot);
      }
    } else {
      msg.robot_info_theirs.emplace_back(merged_robot);
    }
  }

  // チーム1のロボット処理（VisionプライマリでEKFフィルタリング済み）
  for (size_t i = 0; i < vision_robots_1.size() && i < data.robot_info[1].size(); ++i) {
    auto merged_robot = vision_robots_1[i];

    // フィードバック情報をマージ（従来の処理から継承）
    merged_robot.feedback_detected = data.robot_info[1][i].feedback_detected;
    merged_robot.ball_sensor = data.robot_info[1][i].ball_sensor;
    merged_robot.last_ball_sensor_stamp = data.robot_info[1][i].last_ball_sensor_stamp;
    merged_robot.last_feedback_detection_stamp =
      data.robot_info[1][i].last_feedback_detection_stamp;
    merged_robot.detected = merged_robot.vision_detected or merged_robot.feedback_detected;

    // 外部トラッカーとVisionの比較ログ（両方利用可能な場合）
    if (
      merged_robot.vision_detected && i < tracker_robots_1.size() &&
      tracker_robots_1[i].vision_detected) {
      const auto & tracker_robot = tracker_robots_1[i];
      double vision_angle = merged_robot.pose.theta;
      double tracker_angle = tracker_robot.pose.theta;
      double angle_diff = std::abs(vision_angle - tracker_angle);
      // 角度差を[-π, π]範囲に正規化
      if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
      }

      // 角度差が大きい場合（10度以上）はログ出力
      if (angle_diff > M_PI / 18.0) {  // 10度 = π/18 ラジアン
        RCLCPP_WARN(
          node.get_logger(),
          "Team1 Robot %zu: Tracker vs Vision angle difference = %.2f deg (Tracker: %.2f, Vision: "
          "%.2f)",
          i, angle_diff * 180.0 / M_PI, tracker_angle * 180.0 / M_PI, vision_angle * 180.0 / M_PI);
      }
    }

    // Visionデータが無い場合はトラッカーからフォールバック
    if (!merged_robot.vision_detected && i < tracker_robots_1.size()) {
      const auto & tracker_robot = tracker_robots_1[i];
      if (tracker_robot.vision_detected) {
        RCLCPP_DEBUG(
          node.get_logger(), "Team1 Robot %zu: Using tracker fallback (angle: %.2f deg)", i,
          tracker_robot.pose.theta * 180.0 / M_PI);
        merged_robot.pose = tracker_robot.pose;
        merged_robot.velocity = tracker_robot.velocity;
        merged_robot.velocity_norm = tracker_robot.velocity_norm;
        merged_robot.vision_detected = true;
        merged_robot.detected = true;
      }
    }

    if (static_cast<uint8_t>(game_data.our_color) == 1) {
      if (ranges::contains(robot_ids_mask, merged_robot.id)) {
        msg.robot_info_theirs.emplace_back(merged_robot);
      } else {
        msg.robot_info_ours.emplace_back(merged_robot);
      }
    } else {
      msg.robot_info_theirs.emplace_back(merged_robot);
    }
  }

  // 通常モード - 変換なし
  crane_msgs::msg::FieldSize field_info;
  field_info.x = game_data.field_w;
  field_info.y = game_data.field_h;
  msg.field_info = field_info;

  crane_msgs::msg::FieldSize penalty_area_size;
  penalty_area_size.x = game_data.penalty_area_h;
  penalty_area_size.y = game_data.penalty_area_w;
  msg.penalty_area_size = penalty_area_size;

  crane_msgs::msg::FieldSize goal_size;
  goal_size.x = game_data.goal_h;
  goal_size.y = game_data.goal_w;
  msg.goal_size = goal_size;

  // Validation warning for invalid geometry
  if (game_data.field_w <= 0.0 || game_data.field_h <= 0.0) {
    static rclcpp::Time last_warning_time = rclcpp::Clock(RCL_ROS_TIME).now();
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    // Warn every 5 seconds to avoid spam
    if ((now - last_warning_time).seconds() > 5.0) {
      RCLCPP_WARN(
        node.get_logger(),
        "Invalid field geometry in WorldModel: field=%.3fx%.3f, goal=%.3fx%.3f, "
        "penalty_area=%.3fx%.3f",
        game_data.field_w, game_data.field_h, game_data.goal_w, game_data.goal_h,
        game_data.penalty_area_w, game_data.penalty_area_h);
      last_warning_time = now;
    }
  }

  msg.our_goalie_id = game_data.our_goalie_id;
  msg.their_goalie_id = game_data.their_goalie_id;

  msg.play_situation = latest_play_situation;

  // Vision遅延情報をDelayCheckpointに追加
  if (
    vision_processor_->getLastVisionTCapture() > 0.0 &&
    vision_processor_->getLastVisionTSent() > 0.0) {
    auto now = rclcpp::Clock().now();
    std::string vision_delay_info = DelayMonitorWrapper::formatVisionDelayInfo(
      vision_processor_->getLastVisionTCapture(), vision_processor_->getLastVisionTSent(), now);

    DelayMonitorWrapper::addDelayCheckpoint(
      msg.delay_checkpoints, "vision_timestamps", vision_delay_info);
  }

  msg.header.stamp = rclcpp::Clock().now();
  return msg;
}
}  // namespace crane
