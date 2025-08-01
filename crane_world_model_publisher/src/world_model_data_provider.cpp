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
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <robocup_ssl_msgs/msg/robot_id.hpp>
#include <string>
#include <vector>

namespace crane
{

WorldModelDataProvider::WorldModelDataProvider(rclcpp::Node & node) : node(node)
{
  using std::chrono_literals::operator""ms;

  // VisionDataProcessor を初期化
  vision_processor_ = std::make_unique<VisionDataProcessor>(node);
  // data_source_manager_ removed - direct VisionDataProcessor usage

  area_mask.min_corner() << -20., -10.;
  area_mask.max_corner() << 20., 10.;

  vision_processor_->setVisualizationHandler(
    [this](const SSL_GeometryData & geometry_data, bool half_court_mode) {
      if (geometry_visualization_callback_) {
        geometry_visualization_callback_(geometry_data, half_court_mode);
      }
    });

  vision_processor_->setGeometryUpdateHandler([this]() {
    // Update geometry data whenever vision geometry is received
    updateGeometryIfNeeded();
  });

  udp_timer = node.create_wall_timer(10ms, std::bind(&WorldModelDataProvider::on_udp_timer, this));

  // Robot info initialization removed - get directly from VisionDataProcessor

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

      // Legacy robot_info processing removed - handled directly in VisionDataProcessor integration
    });

  // Robot status subscriptions removed - ball sensor data integrated via VisionDataProcessor feedback

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

      if (referee_visualization_callback_) {
        referee_visualization_callback_(msg, game_data.field_w, game_data.field_h);
      }
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

  geometry_initialized = true;
}

// createGameConfiguration() removed - use game_data directly

crane_msgs::msg::WorldModel WorldModelDataProvider::getMsg()
{
  crane_msgs::msg::WorldModel msg;

  // Basic game configuration
  msg.is_yellow = (game_data.our_color == Color::YELLOW);
  msg.on_positive_half = on_positive_half;
  msg.is_emplace_positive_side = is_emplace_positive_side;
  msg.our_max_allowed_bots = game_data.our_max_allowed_bots;
  msg.their_max_allowed_bots = game_data.their_max_allowed_bots;
  msg.our_goalie_id = game_data.our_goalie_id;
  msg.their_goalie_id = game_data.their_goalie_id;
  msg.play_situation = latest_play_situation;

  // Get ball data directly from vision processor
  if (vision_processor_->hasVisionUpdated()) {
    msg.ball_info = vision_processor_->getBallInfo();
  } else {
    // Use default ball info when vision is not available
    msg.ball_info = crane_msgs::msg::BallInfo{};
    RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 5000, "No fresh ball data available");
  }

  // Process robot data for both teams
  std::vector<crane_msgs::msg::RobotInfo> team_0_robots;
  std::vector<crane_msgs::msg::RobotInfo> team_1_robots;

  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    auto vision_robots = vision_processor_->getRobotInfo(team_idx);
    
    // For now, use vision data directly without complex feedback merging
    // Feedback integration is already handled in VisionDataProcessor
    if (team_idx == 0) {
      team_0_robots = vision_robots;
    } else {
      team_1_robots = vision_robots;
    }
  }

  // Classify robots by our team vs their team
  auto [our_robots, their_robots] = classifyRobotsByTeam(team_0_robots, team_1_robots);
  msg.robot_info_ours = our_robots;
  msg.robot_info_theirs = their_robots;

  // Set field information
  msg.field_info = createFieldInfo();
  msg.penalty_area_size = createPenaltyAreaInfo();
  msg.goal_size = createGoalInfo();

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
        game_data.field_w, game_data.field_h, game_data.goal_w,
        game_data.goal_h, game_data.penalty_area_w, game_data.penalty_area_h);
      last_warning_time = now;
    }
  }

  msg.header.stamp = rclcpp::Clock().now();
  return msg;
}
auto WorldModelDataProvider::setVisualizationCallbacks(
  std::function<void(const SSL_GeometryData &, bool)> geometry_callback,
  std::function<void(const robocup_ssl_msgs::msg::Referee &, double, double)> referee_callback)
  -> void
{
  geometry_visualization_callback_ = geometry_callback;
  referee_visualization_callback_ = referee_callback;
}

// Helper method implementations moved from DataSourceManager
auto WorldModelDataProvider::createFieldInfo() -> crane_msgs::msg::FieldSize
{
  crane_msgs::msg::FieldSize field_info;
  field_info.x = game_data.field_w;
  field_info.y = game_data.field_h;
  return field_info;
}

auto WorldModelDataProvider::createPenaltyAreaInfo() -> crane_msgs::msg::FieldSize
{
  crane_msgs::msg::FieldSize penalty_area_size;
  penalty_area_size.x = game_data.penalty_area_h;
  penalty_area_size.y = game_data.penalty_area_w;
  return penalty_area_size;
}

auto WorldModelDataProvider::createGoalInfo() -> crane_msgs::msg::FieldSize
{
  crane_msgs::msg::FieldSize goal_size;
  goal_size.x = game_data.goal_h;
  goal_size.y = game_data.goal_w;
  return goal_size;
}

auto WorldModelDataProvider::mergeRobotInfo(
  const crane_msgs::msg::RobotInfo & vision_robot,
  const crane_msgs::msg::RobotInfo & feedback_robot) -> crane_msgs::msg::RobotInfo
{
  auto merged = vision_robot;

  // Primary data source is vision (with EKF filtering)
  // Merge feedback information
  merged.feedback_detected = feedback_robot.feedback_detected;
  merged.ball_sensor = feedback_robot.ball_sensor;
  merged.last_ball_sensor_stamp = feedback_robot.last_ball_sensor_stamp;
  merged.last_feedback_detection_stamp = feedback_robot.last_feedback_detection_stamp;

  // Combine detection flags
  merged.detected = merged.vision_detected || merged.feedback_detected;

  return merged;
}

auto WorldModelDataProvider::classifyRobotsByTeam(
  const std::vector<crane_msgs::msg::RobotInfo> & robots_team_0,
  const std::vector<crane_msgs::msg::RobotInfo> & robots_team_1)
  -> std::pair<std::vector<crane_msgs::msg::RobotInfo>, std::vector<crane_msgs::msg::RobotInfo>>
{
  std::vector<crane_msgs::msg::RobotInfo> our_robots;
  std::vector<crane_msgs::msg::RobotInfo> their_robots;

  // Classify team 0 robots
  for (const auto & robot : robots_team_0) {
    if (static_cast<uint8_t>(game_data.our_color) == 0) {
      if (
        std::find(robot_ids_mask.begin(), robot_ids_mask.end(), robot.id) !=
        robot_ids_mask.end()) {
        their_robots.push_back(robot);
      } else {
        our_robots.push_back(robot);
      }
    } else {
      their_robots.push_back(robot);
    }
  }

  // Classify team 1 robots
  for (const auto & robot : robots_team_1) {
    if (static_cast<uint8_t>(game_data.our_color) == 1) {
      if (
        std::find(robot_ids_mask.begin(), robot_ids_mask.end(), robot.id) !=
        robot_ids_mask.end()) {
        their_robots.push_back(robot);
      } else {
        our_robots.push_back(robot);
      }
    } else {
      their_robots.push_back(robot);
    }
  }

  return {our_robots, their_robots};
}

}  // namespace crane
