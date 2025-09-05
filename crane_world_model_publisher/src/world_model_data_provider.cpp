// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/world_model_data_provider.hpp"
#include "crane_world_model_publisher/robot_tracker.hpp"

#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/delay_monitor_wrapper.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <robocup_ssl_msgs/msg/robot_id.hpp>
#include <string>
#include <vector>

namespace crane
{

WorldModelDataProvider::WorldModelDataProvider(rclcpp::Node & node)
: node(node),
  our_team_color_(TeamColor::BLUE),
  has_vision_updated_(false),
  has_latest_detection_frame_(false),
  last_t_capture_(0.0),
  last_t_sent_(0.0),
  last_ball_detect_time_(node.get_clock()->now()),
  last_prediction_time_(node.get_clock()->now())
{
  using std::chrono_literals::operator""ms;

  // VisionStreamProcessorの機能を統合：パラメータ設定
  node.declare_parameter("vision_address", config_.vision_address);
  node.declare_parameter("vision_port", config_.vision_port);
  node.declare_parameter("confidence_threshold", config_.confidence_threshold);

  config_.vision_address = node.get_parameter("vision_address").get_value<std::string>();
  config_.vision_port = node.get_parameter("vision_port").get_value<int>();
  config_.confidence_threshold = node.get_parameter("confidence_threshold").get_value<double>();

  // MulticastReceiver初期化
  try {
    multicast_receiver_ =
      std::make_unique<multicast::MulticastReceiver>(config_.vision_address, config_.vision_port);
    RCLCPP_INFO(
      node.get_logger(), "WorldModelDataProvider Vision initialized on %s:%d",
      config_.vision_address.c_str(), config_.vision_port);
  } catch (const std::exception & e) {
    reportError("Failed to initialize vision stream: " + std::string(e.what()));
    RCLCPP_ERROR(node.get_logger(), "Vision initialization failed: %s", e.what());
  }

  robot_tracker_manager_ = std::make_unique<RobotTrackerManager>(node.get_clock());
  RCLCPP_INFO(node.get_logger(), "RobotTrackerManager initialized for EKF-based robot tracking");

  // ロボット情報初期化
  for (int team = 0; team < 2; ++team) {
    robot_info_[team].resize(MAX_ROBOT_COUNT);
    for (size_t i = 0; i < MAX_ROBOT_COUNT; ++i) {
      auto & robot = robot_info_[team][i];
      robot.id = static_cast<uint8_t>(i);
      robot.vision_detected = false;
      robot.feedback_detected = false;
      robot.internal_tracker_detected = false;
      robot.detected = false;
    }
  }

  // ボール情報初期化
  ball_info_.detected = false;
  ball_info_.state = crane_msgs::msg::BallInfo::STOPPED;

  // ボールデータ品質管理初期化
  last_ball_data_ = {Eigen::Vector3d::Zero(), node.get_clock()->now()};
  velocity_history_.clear();

  area_mask.min_corner() << -20., -10.;
  area_mask.max_corner() << 20., 10.;

  udp_timer = node.create_wall_timer(10ms, std::bind(&WorldModelDataProvider::on_udp_timer, this));

  // /play_situationのトピック統計はsession_controllerで取得
  sub_play_situation = node.create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 1,
    [this](const crane_msgs::msg::PlaySituation msg) { latest_play_situation = msg; });

  sub_robot_feedback = node.create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 1, [this](const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg) {
      robot_feedback = *msg;
      auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    });

  node.declare_parameter("team_name", "ibis-ssl");
  game_data.team_name = node.get_parameter("team_name").as_string();

  node.declare_parameter("initial_team_color", "BLUE");
  auto initial_team_color = node.get_parameter("initial_team_color").as_string();
  if (initial_team_color == "BLUE") {
    game_data.our_color = Color::BLUE;
    game_data.their_color = Color::YELLOW;
    our_team_color_ = TeamColor::BLUE;
  } else {
    game_data.our_color = Color::YELLOW;
    game_data.their_color = Color::BLUE;
    our_team_color_ = TeamColor::YELLOW;
  }

  node.declare_parameter("is_emplace_positive_side", true);
  is_emplace_positive_side = node.get_parameter("is_emplace_positive_side").get_value<bool>();

  sub_referee = node.create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 1, [this](const robocup_ssl_msgs::msg::Referee & msg) {
      if (msg.yellow.name == game_data.team_name) {
        // YELLOW
        game_data.our_color = Color::YELLOW;
        game_data.their_color = Color::BLUE;
        our_team_color_ = TeamColor::YELLOW;
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
        our_team_color_ = TeamColor::BLUE;
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
        reportError(what.str());
      }

      if (not msg.designated_position.empty()) {
        data.ball_placement_target_x = msg.designated_position.front().x / 1000.;
        data.ball_placement_target_y = msg.designated_position.front().y / 1000.;
      }

      if (referee_visualization_callback_) {
        referee_visualization_callback_(msg, game_data.field_w, game_data.field_h);
      }
      CraneVisualizerBuffer::publish();
    });
}

auto WorldModelDataProvider::on_udp_timer() -> void
{
  if (!multicast_receiver_) {
    RCLCPP_WARN_THROTTLE(
      node.get_logger(), *node.get_clock(), 5000, "MulticastReceiver is not initialized");
    return;
  }

  int packets_processed = 0;
  while (multicast_receiver_->available()) {
    std::vector<char> raw_packet_data(2048);
    try {
      size_t received = multicast_receiver_->receive(raw_packet_data);
      if (received > 0) {
        packets_processed++;

        try {
          SSL_WrapperPacket packet;
          if (packet.ParseFromArray(raw_packet_data.data(), static_cast<int>(received))) {
            if (packet.has_detection()) {
              processDetectionFrame(packet.detection());
              has_vision_updated_ = true;
            }
            if (packet.has_geometry()) {
              processGeometryData(packet.geometry());
            }
          } else {
            RCLCPP_DEBUG(node.get_logger(), "Failed to parse SSL_WrapperPacket");
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(node.get_logger(), "Packet parsing error: %s", e.what());
        }
      }
    } catch (const std::exception & e) {
      // 受信エラーは無視（非ブロッキング受信のため）
      break;
    }
  }

  // デバッグ用ログ出力（5秒間隔）
  static rclcpp::Time last_debug_log = node.get_clock()->now();
  auto now = node.get_clock()->now();
  if ((now - last_debug_log).seconds() > 5.0) {
    last_debug_log = now;
  }

  if (!geometry_initialized) {
    updateGeometryIfNeeded();
  }
}

auto WorldModelDataProvider::updateGeometryIfNeeded() -> void
{
  const auto & geometry = field_geometry_;
  double field_w = geometry.field_width;
  double field_h = geometry.field_height;

  if (field_w <= 0.0 || field_h <= 0.0) {
    // Vision geometry not yet available, skip update
    return;
  }

  bool geometry_changed = !geometry_initialized || std::abs(game_data.field_w - field_w) > 1e-6 ||
                          std::abs(game_data.field_h - field_h) > 1e-6;

  game_data.field_w = field_w;
  game_data.field_h = field_h;
  game_data.goal_w = geometry.goal_width;
  game_data.goal_h = geometry.goal_height;
  game_data.penalty_area_w = geometry.penalty_area_width;
  game_data.penalty_area_h = geometry.penalty_area_height;

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

  // Get ball data directly from local data
  if (has_vision_updated_) {
    msg.ball_info = ball_info_;
  } else {
    // Use default ball info when vision is not available
    msg.ball_info = crane_msgs::msg::BallInfo{};
    RCLCPP_WARN_THROTTLE(
      node.get_logger(), *node.get_clock(), 5000, "No fresh ball data available");
  }

  std::vector<crane_msgs::msg::RobotInfo> team_0_robots;
  std::vector<crane_msgs::msg::RobotInfo> team_1_robots;

  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    auto vision_robots = robot_info_[team_idx];

    // robot_feedbackデータを統合
    for (auto & robot : vision_robots) {
      // robot_feedbackから対応するロボットの情報を検索
      for (const auto & feedback : robot_feedback.feedback) {
        if (feedback.robot_id == robot.id) {
          // feedbackデータを含むRobotInfoを作成
          crane_msgs::msg::RobotInfo feedback_robot;
          feedback_robot.id = feedback.robot_id;
          feedback_robot.feedback_detected = true;
          feedback_robot.ball_sensor = feedback.ball_sensor;
          feedback_robot.last_ball_sensor_stamp = feedback.received_stamp;
          feedback_robot.last_feedback_detection_stamp = feedback.received_stamp;

          // visionデータとfeedbackデータを統合
          robot = mergeRobotInfo(robot, feedback_robot);
          break;
        }
      }
    }

    if (team_idx == 0) {
      team_0_robots = vision_robots;
    } else {
      team_1_robots = vision_robots;
    }
  }

  // VisionStreamProcessorで既に味方・相手チームに分類済み
  // team_0_robots = 味方チーム, team_1_robots = 相手チーム
  msg.robot_info_ours = team_0_robots;
  msg.robot_info_theirs = team_1_robots;

  msg.field_info.x = game_data.field_w;
  msg.field_info.y = game_data.field_h;

  msg.penalty_area_size.x = game_data.penalty_area_h;
  msg.penalty_area_size.y = game_data.penalty_area_w;

  msg.goal_size.x = game_data.goal_h;
  msg.goal_size.y = game_data.goal_w;

  // Vision遅延情報をDelayCheckpointに追加
  if (last_t_capture_ > 0.0 && last_t_sent_ > 0.0) {
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    std::string vision_delay_info =
      DelayMonitorWrapper::formatVisionDelayInfo(last_t_capture_, last_t_sent_, now);

    DelayMonitorWrapper::addDelayCheckpoint(
      msg.delay_checkpoints, "vision_timestamps", vision_delay_info);
  }

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

  msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
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

auto WorldModelDataProvider::processDetectionFrame(const SSL_DetectionFrame & detection) -> bool
{
  // 最新のSSL_DetectionFrameを保存（detection_frame生成用）
  latest_ssl_detection_frame_ = detection;
  has_latest_detection_frame_ = true;

  // タイムスタンプ更新
  last_t_capture_ = detection.t_capture();
  last_t_sent_ = detection.t_sent();

  // ボール検出処理
  if (!detection.balls().empty()) {
    const auto & ssl_ball = detection.balls().at(0);
    convertBallDetection(ssl_ball);
    last_ball_detect_time_ = node.get_clock()->now();
  } else {
    // ボール未検出時の処理
    auto now = node.get_clock()->now();
    if ((now - last_ball_detect_time_).seconds() > 0.1) {
      ball_info_.detected = false;
      ball_info_.velocity.x = 0.0;
      ball_info_.velocity.y = 0.0;
      ball_info_.velocity.z = 0.0;
    }
  }

  // ロボット検出処理
  // 青チーム
  for (const auto & ssl_robot : detection.robots_blue()) {
    if (ssl_robot.has_robot_id()) {
      uint8_t robot_id = static_cast<uint8_t>(ssl_robot.robot_id());
      if (robot_id < MAX_ROBOT_COUNT) {
        if (std::isfinite(ssl_robot.orientation())) {
          int team_index = (our_team_color_ == TeamColor::BLUE) ? 0 : 1;
          convertRobotDetection(ssl_robot, team_index, robot_id);
        }
      }
    }
  }

  // 黄チーム
  for (const auto & ssl_robot : detection.robots_yellow()) {
    if (ssl_robot.has_robot_id()) {
      uint8_t robot_id = static_cast<uint8_t>(ssl_robot.robot_id());
      if (robot_id < MAX_ROBOT_COUNT) {
        if (std::isfinite(ssl_robot.orientation())) {
          int team_index = (our_team_color_ == TeamColor::YELLOW) ? 0 : 1;
          convertRobotDetection(ssl_robot, team_index, robot_id);
        }
      }
    }
  }
  
  // 古いTrackerの削除（2秒間検出されなかったものを削除）
  robot_tracker_manager_->removeOldTrackers(2.0);

  // 各チーム・各ロボットIDについてTrackerから推定結果を取得
  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    RobotTrackerType tracker_type = (team_idx == 0) ? RobotTrackerType::FRIENDLY : RobotTrackerType::ENEMY;
    
    for (size_t robot_id = 0; robot_id < MAX_ROBOT_COUNT; ++robot_id) {
      auto & robot = robot_info_[team_idx][robot_id];
      
      // Trackerから推定結果を取得
      auto tracker = robot_tracker_manager_->getRobotTracker(static_cast<uint8_t>(robot_id), tracker_type);
      
      if (tracker && tracker->getTrackingConfidence() > 0.2) {  // 最小信頼度閾値
        // Tracker推定結果をrobot_info_に反映（vision_detectedが更新されていない場合のみ）
        if (!robot.vision_detected) {
          auto pos = tracker->getPosition();
          robot.pose.x = pos(0);
          robot.pose.y = pos(1);
          robot.pose.theta = tracker->getTheta();

          auto vel = tracker->getVelocity();
          robot.velocity.x = vel(0);
          robot.velocity.y = vel(1);
          robot.velocity_norm = vel.norm();
        }
        
        // tracking_confidenceベースの内部追跡フラグ
        robot.internal_tracker_detected = tracker->getTrackingConfidence() > 0.5;
        robot.last_tracker_detection_stamp = tracker->getLastUpdateTime();
      } else {
        // Trackerが存在しないまたは信頼度が低い場合
        robot.internal_tracker_detected = false;
        
        // Vision検出もされていない場合は速度を0にリセット
        if (!robot.vision_detected) {
          robot.velocity.x = 0.0;
          robot.velocity.y = 0.0;
          robot.velocity_norm = 0.0;
        }
      }
      
      // 最終的な検出フラグ設定
      robot.detected = robot.vision_detected || robot.feedback_detected || robot.internal_tracker_detected;
    }
  }

  return true;
}

auto WorldModelDataProvider::processGeometryData(const SSL_GeometryData & geometry) -> bool
{
  convertFieldGeometry(geometry);

  if (geometry_visualization_callback_) {
    geometry_visualization_callback_(geometry, false);  // half_court_mode = false
  }

  return true;
}

auto WorldModelDataProvider::convertBallDetection(const SSL_DetectionBall & ssl_ball) -> void
{
  // 座標変換 (mm -> m)
  double x = ssl_ball.x() / 1000.0;
  double y = ssl_ball.y() / 1000.0;
  double z = ssl_ball.has_z() ? ssl_ball.z() / 1000.0 : 0.0;

  Eigen::Vector3d current_pos(x, y, z);
  auto now = node.get_clock()->now();
  double dt = (now - last_ball_data_.second).seconds();

  // データ品質統計更新
  ball_data_quality_.stats.total_detections++;

  // 初回データまたは長時間の空白後はリセット
  if (!ball_data_initialized_ || dt > 1.0) {
    ball_data_initialized_ = true;
    last_ball_data_ = {current_pos, now};
    velocity_history_.clear();

    // 位置のみ更新、速度は0とする
    ball_info_.position.x = x;
    ball_info_.position.y = y;
    ball_info_.position.z = z;
    ball_info_.velocity.x = 0.0;
    ball_info_.velocity.y = 0.0;
    ball_info_.velocity.z = 0.0;
    ball_info_.velocity_norm = 0.0;

    RCLCPP_DEBUG(node.get_logger(), "Ball tracking initialized at (%.3f, %.3f, %.3f)", x, y, z);

    ball_info_.detected = true;
    ball_info_.state = crane_msgs::msg::BallInfo::STOPPED;

    // Vision情報更新
    ball_info_.vision.stamp = now;
    ball_info_.vision.pos.x = x;
    ball_info_.vision.pos.y = y;
    ball_info_.vision.pos.z = z;
    return;
  }

  // 異常な位置変化をチェック
  if (!validateBallPositionChange(current_pos, dt)) {
    ball_data_quality_.stats.outlier_rejections++;
    ball_data_quality_.stats.position_jumps++;

    // テレポート候補の場合は拒否統計も更新
    if ((current_pos - last_ball_data_.first).norm() > ball_data_quality_.teleport_threshold) {
      ball_data_quality_.stats.teleport_rejected++;
    }

    RCLCPP_WARN(
      node.get_logger(),
      "Ball position jump rejected: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f), dt=%.3fs",
      last_ball_data_.first.x(), last_ball_data_.first.y(), last_ball_data_.first.z(), x, y, z, dt);
    return;
  }

  // 速度計算（時間差が十分ある場合のみ）
  if (dt >= ball_data_quality_.min_dt) {
    Eigen::Vector3d raw_velocity = (current_pos - last_ball_data_.first) / dt;

    // 速度妥当性チェック
    if (!validateBallVelocity(raw_velocity, dt)) {
      ball_data_quality_.stats.outlier_rejections++;
      ball_data_quality_.stats.velocity_outliers++;

      RCLCPP_WARN(
        node.get_logger(), "Ball velocity outlier rejected: %.3fm/s (raw), dt=%.3fs",
        raw_velocity.norm(), dt);
      return;
    }

    // 速度平滑化
    Eigen::Vector3d smoothed_velocity = smoothBallVelocity(raw_velocity);

    // 速度情報更新
    ball_info_.velocity.x = smoothed_velocity(0);
    ball_info_.velocity.y = smoothed_velocity(1);
    ball_info_.velocity.z = smoothed_velocity(2);
    ball_info_.velocity_norm = smoothed_velocity.norm();

    last_ball_data_ = {current_pos, now};
  }

  // 位置情報更新
  ball_info_.position.x = x;
  ball_info_.position.y = y;
  ball_info_.position.z = z;

  // Vision情報更新
  ball_info_.vision.stamp = now;
  ball_info_.vision.pos.x = x;
  ball_info_.vision.pos.y = y;
  ball_info_.vision.pos.z = z;

  ball_info_.detected = true;
  ball_info_.state = crane_msgs::msg::BallInfo::ROLLING;  // 簡易状態設定

  // 統計情報定期更新
  updateQualityStatistics();
}

auto WorldModelDataProvider::convertRobotDetection(
  const SSL_DetectionRobot & ssl_robot, int team_index, uint8_t robot_id) -> void
{
  if (team_index >= 2 || robot_id >= robot_info_[team_index].size()) {
    return;
  }

  // 座標変換（mm -> m）
  double x = ssl_robot.x() / 1000.0;
  double y = ssl_robot.y() / 1000.0;
  double theta = crane::normalizeAngle(ssl_robot.orientation());

  // Trackerタイプの決定（team_index 0 = 味方, 1 = 相手）
  RobotTrackerType tracker_type = (team_index == 0) ? RobotTrackerType::FRIENDLY : RobotTrackerType::ENEMY;

  // RobotTrackerManagerにVision検出を送信
  Eigen::Vector3d robot_pose(x, y, theta);
  auto now = node.get_clock()->now();
  robot_tracker_manager_->processVisionDetection(robot_id, tracker_type, robot_pose, now);

  // Trackerから推定結果を取得してRobotInfoを更新
  auto tracker = robot_tracker_manager_->getRobotTracker(robot_id, tracker_type);
  if (tracker) {
    auto & robot = robot_info_[team_index][robot_id];
    
    // Tracker推定結果をRobotInfoに反映
    auto pos = tracker->getPosition();
    robot.pose.x = pos(0);
    robot.pose.y = pos(1);
    robot.pose.theta = tracker->getTheta();

    auto vel = tracker->getVelocity();
    robot.velocity.x = vel(0);
    robot.velocity.y = vel(1);
    robot.velocity_norm = vel.norm();

    // Vision検出フラグ更新
    robot.vision_detected = true;
    robot.vision.stamp = now;
    robot.vision.pose.x = x;
    robot.vision.pose.y = y;
    robot.vision.pose.theta = theta;

    // tracking_confidenceベースの検出判定
    double tracking_confidence = tracker->getTrackingConfidence();
    robot.internal_tracker_detected = tracking_confidence > 0.5;
    robot.detected = robot.vision_detected || robot.feedback_detected || robot.internal_tracker_detected;
    robot.last_tracker_detection_stamp = now;
  }
}

auto WorldModelDataProvider::convertFieldGeometry(const SSL_GeometryData & ssl_geometry) -> void
{
  if (!ssl_geometry.has_field()) {
    return;
  }

  const auto & field = ssl_geometry.field();

  field_geometry_.field_width = field.field_length() / 1000.0;  // mm -> m
  field_geometry_.field_height = field.field_width() / 1000.0;
  field_geometry_.goal_width = field.goal_width() / 1000.0;
  field_geometry_.goal_height = field.goal_depth() / 1000.0;

  // ペナルティエリア寸法（標準SSL値または計算）
  if (field.has_penalty_area_depth()) {
    field_geometry_.penalty_area_height = field.penalty_area_depth() / 1000.0;
  } else {
    field_geometry_.penalty_area_height = field_geometry_.goal_width;
  }

  if (field.has_penalty_area_width()) {
    field_geometry_.penalty_area_width = field.penalty_area_width() / 1000.0;
  } else {
    field_geometry_.penalty_area_width = field_geometry_.goal_width * 2.0;
  }

  field_geometry_.center_circle_radius = 0.5;  // 標準SSL値
  field_geometry_.is_valid = true;
}

auto WorldModelDataProvider::reportError(const std::string & error_message) -> void
{
  RCLCPP_WARN(node.get_logger(), "WorldModelDataProvider error: %s", error_message.c_str());
}

auto WorldModelDataProvider::validateBallPositionChange(const Eigen::Vector3d & new_pos, double dt)
  -> bool
{
  if (!ball_data_initialized_ || dt <= 0.0) {
    resetTeleportTracking();  // 初回は追跡リセット
    return true;              // 初回または無効な時間差は常に許可
  }

  double distance = (new_pos - last_ball_data_.first).norm();
  double max_allowed_distance = ball_data_quality_.max_position_jump;

  // 時間に応じた最大許容移動距離を計算（物理的上限速度を考慮）
  double time_based_max_distance = ball_data_quality_.max_velocity * dt * 1.2;  // 20%のマージン
  max_allowed_distance = std::min(max_allowed_distance, time_based_max_distance);

  // 通常の物理的移動範囲内の場合
  if (distance <= max_allowed_distance) {
    resetTeleportTracking();  // 通常移動時は追跡リセット
    return true;
  }

  // 大きな位置ジャンプの場合：テレポート判定
  if (
    distance > ball_data_quality_.teleport_threshold ||
    dt > ball_data_quality_.long_gap_threshold) {
    return handleTeleportCandidate(new_pos, node.get_clock()->now());
  }

  // 中程度のジャンプは拒否（ノイズの可能性）
  return false;
}

auto WorldModelDataProvider::validateBallVelocity(
  const Eigen::Vector3d & velocity, double actual_dt) -> bool
{
  double speed = velocity.norm();

  // 詳細デバッグログ: 速度判定の詳細情報
  RCLCPP_DEBUG(
    node.get_logger(), "[VELOCITY_DEBUG] Speed check: speed=%.3fm/s, max_velocity=%.3fm/s", speed,
    ball_data_quality_.max_velocity);

  if (speed > ball_data_quality_.max_velocity) {
    RCLCPP_WARN(
      node.get_logger(), "[VELOCITY_REJECT] Speed limit exceeded: %.3fm/s > %.3fm/s", speed,
      ball_data_quality_.max_velocity);
    return false;
  }

  // 前回速度との加速度チェック（履歴がある場合）
  if (!velocity_history_.empty() && velocity_history_.size() >= 2) {
    Eigen::Vector3d prev_velocity = velocity_history_.back();

    // 実際の時間差を使用（最小値でクランプ）
    double dt = std::max(actual_dt, ball_data_quality_.min_dt);

    double acceleration = (velocity - prev_velocity).norm() / dt;

    // 詳細デバッグログ: 加速度判定の詳細情報
    RCLCPP_DEBUG(
      node.get_logger(),
      "[ACCEL_DEBUG] Accel check: prev_vel=%.3fm/s, curr_vel=%.3fm/s, actual_dt=%.3fs, "
      "used_dt=%.3fs, accel=%.1fm/s², max_accel=%.1fm/s²",
      prev_velocity.norm(), speed, actual_dt, dt, acceleration,
      ball_data_quality_.max_acceleration);

    if (acceleration > ball_data_quality_.max_acceleration) {
      ball_data_quality_.stats.acceleration_outliers++;
      RCLCPP_WARN(
        node.get_logger(),
        "[ACCEL_REJECT] Acceleration limit exceeded: accel=%.1fm/s² > %.1fm/s² "
        "(prev=%.3f→curr=%.3f, actual_dt=%.3f, used_dt=%.3f)",
        acceleration, ball_data_quality_.max_acceleration, prev_velocity.norm(), speed, actual_dt,
        dt);
      return false;
    }
  }

  RCLCPP_DEBUG(node.get_logger(), "[VELOCITY_ACCEPT] Velocity accepted: %.3fm/s", speed);
  return true;
}

auto WorldModelDataProvider::smoothBallVelocity(const Eigen::Vector3d & raw_velocity)
  -> Eigen::Vector3d
{
  // 移動平均フィルタ
  velocity_history_.push_back(raw_velocity);

  if (velocity_history_.size() > ball_data_quality_.smoothing_window) {
    velocity_history_.pop_front();
  }

  if (velocity_history_.empty()) {
    return raw_velocity;
  }

  // 平均計算
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto & vel : velocity_history_) {
    sum += vel;
  }

  return sum / static_cast<double>(velocity_history_.size());
}

auto WorldModelDataProvider::updateQualityStatistics() -> void
{
  // 拒否率を定期的に計算
  if (ball_data_quality_.stats.total_detections > 0) {
    ball_data_quality_.stats.rejection_rate =
      static_cast<double>(ball_data_quality_.stats.outlier_rejections) /
      static_cast<double>(ball_data_quality_.stats.total_detections);
  }

  // 100検出ごとに統計をログ出力
  if (ball_data_quality_.stats.total_detections % 100 == 0) {
    // RCLCPP_INFO(
    //   node.get_logger(),
    //   "Ball quality stats: total=%zu, rejected=%zu (%.1f%%), pos_jumps=%zu, vel_outliers=%zu, "
    //   "acc_outliers=%zu, teleport_ok=%zu, teleport_ng=%zu",
    //   ball_data_quality_.stats.total_detections, ball_data_quality_.stats.outlier_rejections,
    //   ball_data_quality_.stats.rejection_rate * 100.0, ball_data_quality_.stats.position_jumps,
    //   ball_data_quality_.stats.velocity_outliers, ball_data_quality_.stats.acceleration_outliers,
    //   ball_data_quality_.stats.teleport_accepted, ball_data_quality_.stats.teleport_rejected);
  }
}

auto WorldModelDataProvider::handleTeleportCandidate(
  const Eigen::Vector3d & new_pos, const rclcpp::Time & now) -> bool
{
  // 新しいテレポート候補の場合
  if (
    ball_data_quality_.teleport_candidate.consecutive_detections == 0 ||
    (new_pos - ball_data_quality_.teleport_candidate.position).norm() >
      ball_data_quality_.stability_radius) {
    ball_data_quality_.teleport_candidate.position = new_pos;
    ball_data_quality_.teleport_candidate.first_detected = now;
    ball_data_quality_.teleport_candidate.consecutive_detections = 1;
    ball_data_quality_.teleport_candidate.is_stable = false;

    RCLCPP_INFO(
      node.get_logger(), "Ball teleport candidate detected: (%.3f,%.3f,%.3f) - tracking stability",
      new_pos.x(), new_pos.y(), new_pos.z());

    return false;  // 初回は受け入れない、安定性確認待ち
  }

  // 既存候補の安定性確認
  if (
    (new_pos - ball_data_quality_.teleport_candidate.position).norm() <=
    ball_data_quality_.stability_radius) {
    ball_data_quality_.teleport_candidate.consecutive_detections++;

    // 十分な連続検出で安定と判定
    if (
      ball_data_quality_.teleport_candidate.consecutive_detections >=
      ball_data_quality_.stability_frames) {
      ball_data_quality_.teleport_candidate.is_stable = true;
      ball_data_quality_.stats.teleport_accepted++;

      RCLCPP_INFO(
        node.get_logger(),
        "Ball teleport accepted: (%.3f,%.3f,%.3f) -> (%.3f,%.3f,%.3f) after %zu stable detections",
        last_ball_data_.first.x(), last_ball_data_.first.y(), last_ball_data_.first.z(),
        new_pos.x(), new_pos.y(), new_pos.z(),
        ball_data_quality_.teleport_candidate.consecutive_detections);

      resetTeleportTracking();
      return true;  // テレポート受け入れ
    }

    return false;  // まだ安定性確認中
  }

  // 候補位置から離れた場合：新しい候補として扱う
  ball_data_quality_.teleport_candidate.position = new_pos;
  ball_data_quality_.teleport_candidate.first_detected = now;
  ball_data_quality_.teleport_candidate.consecutive_detections = 1;
  ball_data_quality_.teleport_candidate.is_stable = false;

  return false;
}

auto WorldModelDataProvider::resetTeleportTracking() -> void
{
  ball_data_quality_.teleport_candidate.consecutive_detections = 0;
  ball_data_quality_.teleport_candidate.is_stable = false;
}
}  // namespace crane
