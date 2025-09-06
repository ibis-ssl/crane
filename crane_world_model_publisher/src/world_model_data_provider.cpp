// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/world_model_data_provider.hpp"

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
  has_tracked_frame_updated_(false),
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
  // Tracker/legacy切替パラメータ
  node.declare_parameter("tracker_address", std::string("224.5.23.2"));
  node.declare_parameter("tracker_port", 10010);

  config_.vision_address = node.get_parameter("vision_address").get_value<std::string>();
  config_.vision_port = node.get_parameter("vision_port").get_value<int>();
  config_.confidence_threshold = node.get_parameter("confidence_threshold").get_value<double>();


  // MulticastReceiver初期化（Vision UDP）
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

  // MulticastReceiver初期化（Tracker UDP）
  try {
    auto tracker_addr = node.get_parameter("tracker_address").get_value<std::string>();
    auto tracker_port = node.get_parameter("tracker_port").get_value<int>();
    tracker_receiver_ = std::make_unique<multicast::MulticastReceiver>(tracker_addr, tracker_port);
    RCLCPP_INFO(
      node.get_logger(), "WorldModelDataProvider Tracker initialized on %s:%d",
      tracker_addr.c_str(), tracker_port);
  } catch (const std::exception & e) {
    reportError("Failed to initialize tracker stream: " + std::string(e.what()));
    RCLCPP_ERROR(node.get_logger(), "Tracker initialization failed: %s", e.what());
  }

  // ロボット情報初期化
  for (int team = 0; team < 2; ++team) {
    robot_info_[team].resize(MAX_ROBOT_COUNT);
    for (size_t i = 0; i < MAX_ROBOT_COUNT; ++i) {
      auto & robot = robot_info_[team][i];
      robot.id = static_cast<uint8_t>(i);
      robot.vision_detected = false;
      robot.feedback_detected = false;
      robot.tracker_detected = false;
      robot.detected = false;
    }
  }

  // ボール情報初期化
  ball_info_.tracker_detected = false;
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

  // direct UDP from Tracker; no ROS topic subscription
}

auto WorldModelDataProvider::on_udp_timer() -> void
{
  if (!multicast_receiver_) {
    RCLCPP_WARN_THROTTLE(
      node.get_logger(), *node.get_clock(), 5000, "MulticastReceiver is not initialized");
    return;
  }

  int packets_processed = 0;
  while (multicast_receiver_ && multicast_receiver_->available()) {
    std::vector<char> raw_packet_data(2048);
    try {
      size_t received = multicast_receiver_->receive(raw_packet_data);
      if (received > 0) {
        packets_processed++;

        try {
          SSL_WrapperPacket packet;
          if (packet.ParseFromArray(raw_packet_data.data(), static_cast<int>(received))) {
            processDetectionFrame(packet.detection());
            has_vision_updated_ = true;
            RCLCPP_INFO_THROTTLE(
              node.get_logger(), *node.get_clock(), 5000, "Vision data received: frame=%u",
              packet.detection().frame_number());
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

  // Process Tracker UDP packets
  if (tracker_receiver_) {
    while (tracker_receiver_->available()) {
      std::vector<char> raw_packet_data(2048);
      try {
        size_t received = tracker_receiver_->receive(raw_packet_data);
        if (received > 0) {
          TrackerWrapperPacket wrapper_packet;
          if (wrapper_packet.ParseFromArray(raw_packet_data.data(), static_cast<int>(received))) {
            if (wrapper_packet.has_tracked_frame()) {
              // auto tracked_frame_msg = parseTrackedFrameFromWrapper(wrapper_packet);
              latest_tracked_frame = wrapper_packet.tracked_frame();
              has_tracked_frame_updated_ = true;
              RCLCPP_INFO_THROTTLE(
                node.get_logger(), *node.get_clock(), 5000, "Tracker data received: frame=%u",
                wrapper_packet.tracked_frame().frame_number());
              processTrackedFrame(wrapper_packet.tracked_frame());
            }
          }
        }
      } catch (const std::exception & e) {
        // ignore per non-blocking receive
        break;
      }
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
  if (has_vision_updated_ || has_tracked_frame_updated_) {
    msg.ball_info = ball_info_;
  } else {
    // Use default ball info when vision is not available
    msg.ball_info = crane_msgs::msg::BallInfo{};
    RCLCPP_WARN_THROTTLE(
      node.get_logger(), *node.get_clock(), 5000, "No fresh ball data available");
  }

  auto current_time = rclcpp::Clock(RCL_ROS_TIME).now();
  constexpr double FEEDBACK_TIMEOUT_SECONDS = 1.0;

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

  // チーム色に基づいて敵味方を正しく配置
  // team_0_index=0は味方として設定済み, team_1_index=1は敵として設定済み
  if (our_team_color_ == TeamColor::BLUE) {
    // BLUE=味方の場合: team_0が味方, team_1が敵
    msg.robot_info_ours = team_0_robots;
    msg.robot_info_theirs = team_1_robots;
  } else {
    // YELLOW=味方の場合: team_1が味方, team_0が敵
    msg.robot_info_ours = team_1_robots;
    msg.robot_info_theirs = team_0_robots;
  }

  // チーム配置確認ログ
  RCLCPP_DEBUG_THROTTLE(
    node.get_logger(), *node.get_clock(), 5000,
    "Team assignment: our_color=%s, ours=%zu robots, theirs=%zu robots",
    (our_team_color_ == TeamColor::BLUE) ? "BLUE" : "YELLOW", msg.robot_info_ours.size(),
    msg.robot_info_theirs.size());

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
    updateBallInfoByDetectionBall(ball_info_, ssl_ball);
    last_ball_detect_time_ = node.get_clock()->now();
  } else {
    // ボール未検出時の処理
    auto now = node.get_clock()->now();
    if ((now - last_ball_detect_time_).seconds() > 0.1) {
      ball_info_.vision.stamp.sec = 0;
      ball_info_.vision.stamp.nanosec = 0;
      ball_info_.velocity.x = 0.0;
      ball_info_.velocity.y = 0.0;
      ball_info_.velocity.z = 0.0;
    }
  }

  // ロボット検出は現在TrackedFrameで処理される
  // Vision detection frameは主にgeometry情報のために保持

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

auto WorldModelDataProvider::updateBallInfoByDetectionBall(crane_msgs::msg::BallInfo & ball_info, const SSL_DetectionBall & ssl_ball) -> void
{
  // 座標変換 (mm -> m)
  double x = ssl_ball.x() / 1000.0;
  double y = ssl_ball.y() / 1000.0;
  double z = ssl_ball.has_z() ? ssl_ball.z() / 1000.0 : 0.0;

  auto now = node.get_clock()->now();

  ball_info.vision.stamp = now;
  ball_info.vision.pos.x = x;
  ball_info.vision.pos.y = y;
  ball_info.vision.pos.z = z;
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

auto WorldModelDataProvider::processTrackedFrame(
  const TrackedFrame & tracked_frame) -> void
{
  auto now = node.get_clock()->now();

  // ボール情報の処理
  if (!tracked_frame.balls().empty()) {
    const auto & tracked_ball = tracked_frame.balls()[0];  // 最初のボールをプライマリとする
    crane_msgs::msg::BallInfo ball_info;
    updateBallInfoByTrackedBall(ball_info, tracked_ball);
    ball_info_.tracker_detected = true;
    last_ball_detect_time_ = now;
  } else {
    // ボール未検出時の処理
    if ((now - last_ball_detect_time_).seconds() > 0.1) {
      ball_info_.tracker_detected = false;
      ball_info_.velocity.x = 0.0;
      ball_info_.velocity.y = 0.0;
      ball_info_.velocity.z = 0.0;
    }
  }

  // ロボット情報の処理
  // 全チーム・全ロボットIDをリセット
  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    for (auto & robot : robot_info_[team_idx]) {
      robot.vision_detected = false;
      robot.detected = false;
    }
  }

  // TrackedRobotから情報を変換
  for (const auto & tracked_robot : tracked_frame.robots()) {
    uint8_t robot_id = static_cast<uint8_t>(tracked_robot.robot_id().id());
    if (robot_id >= MAX_ROBOT_COUNT) continue;
    int team_index =
      (tracked_robot.robot_id().team() == robocup_ssl_msgs::msg::RobotId::TEAM_COLOR_YELLOW)
        ? static_cast<int>(Color::YELLOW)
        : static_cast<int>(Color::BLUE);

    auto & robot = robot_info_[team_index][robot_id];
    updateRobotInfoByTrackedRobot(robot, tracked_robot);
    robot.vision_detected = true;
    robot.detected = true;
    robot.vision.stamp = now;
  }
}

auto WorldModelDataProvider::updateBallInfoByTrackedBall(
  crane_msgs::msg::BallInfo & ball_info, const TrackedBall & tracked_ball) -> void
{
  auto now = node.get_clock()->now();

  // 位置情報
  ball_info.position.x = tracked_ball.pos().x();
  ball_info.position.y = tracked_ball.pos().y();
  ball_info.position.z = tracked_ball.pos().z();

  // 速度情報（オプション）
  if (tracked_ball.has_vel()) {
    const auto & vel = tracked_ball.vel();
    ball_info.velocity.x = vel.x();
    ball_info.velocity.y = vel.y();
    ball_info.velocity.z = vel.z();
    ball_info.velocity_norm = std::hypot(vel.x(), vel.y(), vel.z());
  } else {
    ball_info.velocity.x = 0.0;
    ball_info.velocity.y = 0.0;
    ball_info.velocity.z = 0.0;
    ball_info.velocity_norm = 0.0;
  }

  // ボール状態の決定（速度に基づく簡易判定）
  if (ball_info.velocity_norm < 0.1) {
    ball_info.state = crane_msgs::msg::BallInfo::STOPPED;
  } else {
    ball_info.state = crane_msgs::msg::BallInfo::ROLLING;
  }

  ball_info.tracker_detected = true;
}

auto WorldModelDataProvider::updateRobotInfoByTrackedRobot(crane_msgs::msg::RobotInfo & robot_info,
  const TrackedRobot & tracked_robot)
  -> void
{
  auto now = node.get_clock()->now();
  robot_info.id = static_cast<uint8_t>(tracked_robot.robot_id().id());

  // 位置・姿勢情報
  robot_info.pose.x = tracked_robot.pos().x();
  robot_info.pose.y = tracked_robot.pos().y();
  robot_info.pose.theta = crane::normalizeAngle(tracked_robot.orientation());

  // 速度情報（オプション）
  if (tracked_robot.has_vel()) {
    const auto & vel = tracked_robot.vel();
    robot_info.velocity.x = vel.x();
    robot_info.velocity.y = vel.y();
    robot_info.velocity_norm = std::hypot(vel.x(), vel.y());
  } else {
    robot_info.velocity.x = 0.0;
    robot_info.velocity.y = 0.0;
    robot_info.velocity_norm = 0.0;
  }

  // 検出フラグ
  robot_info.tracker_detected = true;
}
}  // namespace crane
