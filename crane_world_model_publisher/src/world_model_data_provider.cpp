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
  last_t_capture_(0.0),
  last_t_sent_(0.0),
  last_ball_detect_time_(node.get_clock()->now()),
  last_prediction_time_(node.get_clock()->now()),
  direct_detection_frame_publisher_(nullptr)
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
      auto now = rclcpp::Clock().now();
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
    RCLCPP_INFO(
      node.get_logger(), "Vision status: running=%s, updated=%s",
      multicast_receiver_ ? "true" : "false", has_vision_updated_ ? "true" : "false");
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
    auto now = rclcpp::Clock().now();
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

  msg.header.stamp = rclcpp::Clock().now();
  return msg;
}

auto WorldModelDataProvider::getLatestDetectionFrame() const
  -> robocup_ssl_msgs::msg::DetectionFrame
{
  if (!has_latest_detection_frame_) {
    return robocup_ssl_msgs::msg::DetectionFrame{};
  }
  return parseDetectionFrameFromSSL(latest_ssl_detection_frame_);
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

  // 高速detection_frameパブリッシュ（vision_componentと同じ構造）
  if (direct_detection_frame_publisher_) {
    auto detection_frame_msg = parseDetectionFrameFromSSL(detection);
    direct_detection_frame_publisher_->publish(detection_frame_msg);
  }

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

  // 検出されなかったロボットの速度を0にリセット（停止検知）
  auto now = node.get_clock()->now();
  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    for (size_t robot_id = 0; robot_id < MAX_ROBOT_COUNT; ++robot_id) {
      auto & robot = robot_info_[team_idx][robot_id];
      auto & history = robot_history_[team_idx][robot_id];

      if (!robot.vision_detected) {
        // 検出されなかったロボットの処理
        const double VISIBILITY_DECREMENT = 0.05;  // 非検出時の減少量
        history.visibility = std::max(0.0, history.visibility - VISIBILITY_DECREMENT);

        // internal_tracker_detectedは可視性の閾値判定のみで設定
        robot.internal_tracker_detected = history.visibility >= 0.5;

        if (history.is_initialized) {
          double dt = (now - history.last_update_time).seconds();

          // 検出されなくなってから100ms以上経過したら速度を0にする
          if (dt > 0.1) {
            robot.velocity.x = 0.0;
            robot.velocity.y = 0.0;
            robot.velocity_norm = 0.0;
          }
          // robot.detectedはvisibilityベースで管理し、データの連続性を保つ
          robot.detected =
            robot.vision_detected || robot.feedback_detected || robot.internal_tracker_detected;
        } else {
          robot.detected =
            robot.vision_detected || robot.feedback_detected || robot.internal_tracker_detected;
        }
      }
    }
  }

  // 全ロボットに対してタイムアウト判定（1/30秒 = 約33ms）
  const double VISION_TIMEOUT_SEC = 1.0 / 30.0;  // 約33ms
  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    for (size_t robot_id = 0; robot_id < MAX_ROBOT_COUNT; ++robot_id) {
      auto & robot = robot_info_[team_idx][robot_id];
      auto & history = robot_history_[team_idx][robot_id];

      // last_vision_detection_timeが有効な場合のみタイムアウト判定
      if (history.last_vision_detection_time.nanoseconds() > 0) {
        double time_since_last_detection = (now - history.last_vision_detection_time).seconds();
        if (time_since_last_detection > VISION_TIMEOUT_SEC) {
          robot.vision_detected = false;
        }
      } else {
        // 初期状態では検出されていない
        robot.vision_detected = false;
      }
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

  // 位置更新
  ball_info_.position.x = x;
  ball_info_.position.y = y;
  ball_info_.position.z = z;

  // Vision情報更新
  ball_info_.vision.stamp = node.get_clock()->now();
  ball_info_.vision.pos.x = x;
  ball_info_.vision.pos.y = y;
  ball_info_.vision.pos.z = z;

  // 速度計算
  static std::pair<Eigen::Vector3d, rclcpp::Time> last_ball_data = {
    Eigen::Vector3d::Zero(), node.get_clock()->now()};
  auto now = node.get_clock()->now();
  double dt = (now - last_ball_data.second).seconds();

  if (dt > 0.001) {  // 1ms以上の差分のみ計算
    Eigen::Vector3d current_pos(x, y, z);
    Eigen::Vector3d vel = (current_pos - last_ball_data.first) / dt;
    ball_info_.velocity.x = vel(0);
    ball_info_.velocity.y = vel(1);
    ball_info_.velocity.z = vel(2);
    ball_info_.velocity_norm = vel.norm();
    last_ball_data = {current_pos, now};
  }

  ball_info_.detected = true;
  ball_info_.state = crane_msgs::msg::BallInfo::ROLLING;  // 簡易状態設定
}

auto WorldModelDataProvider::convertRobotDetection(
  const SSL_DetectionRobot & ssl_robot, int team_index, uint8_t robot_id) -> void
{
  if (team_index >= 2 || robot_id >= robot_info_[team_index].size()) {
    return;
  }

  auto & robot = robot_info_[team_index][robot_id];

  // 座標変換
  double x = ssl_robot.x() / 1000.0;
  double y = ssl_robot.y() / 1000.0;
  double theta = crane::normalizeAngle(ssl_robot.orientation());

  // 位置・姿勢更新
  robot.pose.x = x;
  robot.pose.y = y;
  robot.pose.theta = theta;

  // チャタリング抑制を含む検出フラグ更新
  auto & history = robot_history_[team_index][robot_id];
  auto now = node.get_clock()->now();

  // SSL-Visionで検出された時刻を記録
  history.last_vision_detection_time = now;
  robot.vision_detected = true;

  // visionフィールドの更新
  robot.vision.stamp = now;
  robot.vision.pose.x = x;
  robot.vision.pose.y = y;
  robot.vision.pose.theta = theta;

  const double VISIBILITY_INCREMENT = 0.1;  // 検出時の上昇量
  history.visibility = std::min(1.0, history.visibility + VISIBILITY_INCREMENT);

  robot.internal_tracker_detected = history.visibility >= 0.5;
  robot.detected =
    robot.vision_detected || robot.feedback_detected || robot.internal_tracker_detected;
  robot.last_tracker_detection_stamp = now;

  // 速度計算（時間微分）
  if (history.is_initialized) {
    double dt = (now - history.last_update_time).seconds();

    if (dt > 0.001) {  // 1ms以上の差分のみ計算
      Eigen::Vector3d current_pos(x, y, theta);
      Eigen::Vector3d position_diff = current_pos - history.last_position;

      // 角度差の正規化（-π to π）
      position_diff(2) = crane::normalizeAngle(position_diff(2));

      Eigen::Vector3d vel = position_diff / dt;

      robot.velocity.x = vel(0);
      robot.velocity.y = vel(1);
      robot.velocity_norm = vel.head<2>().norm();

      history.last_position = current_pos;
      history.last_update_time = now;
    }
  } else {
    // 初回は速度0で初期化
    robot.velocity.x = 0.0;
    robot.velocity.y = 0.0;
    robot.velocity_norm = 0.0;

    // 履歴の継続性チェック：大きな座標ジャンプの防止
    if (history.last_update_time.nanoseconds() > 0) {
      double time_since_last = (now - history.last_update_time).seconds();

      // 前回位置からの距離チェック
      Eigen::Vector3d prev_pos(history.last_position);
      double distance_jump = sqrt(pow(x - prev_pos(0), 2) + pow(y - prev_pos(1), 2));

      // 大きなジャンプ（1m以上）で短時間（5秒以内）の場合は履歴を継続
      if (distance_jump > 1.0 && time_since_last < 5.0) {
        RCLCPP_WARN(
          node.get_logger(),
          "Robot[%d][%d] large jump detected: %.3fm after %.3fs - keeping history continuity",
          team_index, robot_id, distance_jump, time_since_last);

        // 履歴を継続し、前回位置からの速度を計算
        if (time_since_last > 0.001) {
          Eigen::Vector3d current_pos(x, y, theta);
          Eigen::Vector3d position_diff = current_pos - history.last_position;
          position_diff(2) = crane::normalizeAngle(position_diff(2));
          Eigen::Vector3d vel = position_diff / time_since_last;

          robot.velocity.x = vel(0);
          robot.velocity.y = vel(1);
          robot.velocity_norm = vel.head<2>().norm();
        }

        history.last_position = Eigen::Vector3d(x, y, theta);
        history.last_update_time = now;
        return;  // is_initializedはtrueのまま継続
      }
    }

    // 通常の初期化処理
    robot.velocity.x = 0.0;
    robot.velocity.y = 0.0;
    robot.velocity_norm = 0.0;

    history.last_position = Eigen::Vector3d(x, y, theta);
    history.last_update_time = now;
    history.is_initialized = true;
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

auto WorldModelDataProvider::parseDetectionFrameFromSSL(
  const SSL_DetectionFrame & ssl_detection) const -> robocup_ssl_msgs::msg::DetectionFrame
{
  robocup_ssl_msgs::msg::DetectionFrame detection_frame_msg;

  detection_frame_msg.frame_number = ssl_detection.frame_number();
  detection_frame_msg.t_capture = ssl_detection.t_capture();
  detection_frame_msg.t_sent = ssl_detection.t_sent();
  detection_frame_msg.camera_id = ssl_detection.camera_id();

  // Parse balls（vision_componentと同じ構造）
  for (const auto & ball : ssl_detection.balls()) {
    robocup_ssl_msgs::msg::DetectionBall ball_msg;
    ball_msg.confidence = ball.confidence();
    if (ball.has_area()) {
      ball_msg.area = ball.area();
    } else {
      ball_msg.area = 100;  // invalid value
    }
    ball_msg.x = ball.x() / 1000.0;
    ball_msg.y = ball.y() / 1000.0;
    if (ball.has_z()) {
      ball_msg.z = ball.z() / 1000.0;
    } else {
      ball_msg.z = 0.0;  // invalid value
    }
    ball_msg.pixel_x = ball.pixel_x();
    ball_msg.pixel_y = ball.pixel_y();

    detection_frame_msg.balls.push_back(ball_msg);
  }

  // Parse yellow robots（vision_componentと同じ構造）
  for (const auto & robot : ssl_detection.robots_yellow()) {
    robocup_ssl_msgs::msg::DetectionRobot robot_msg;
    robot_msg.confidence = robot.confidence();
    if (robot.has_robot_id()) {
      robot_msg.robot_id = robot.robot_id();
    } else {
      robot_msg.robot_id = 100;  // invalid value
    }
    robot_msg.x = robot.x() / 1000.0;
    robot_msg.y = robot.y() / 1000.0;
    if (robot.has_orientation()) {
      robot_msg.orientation = robot.orientation();
    } else {
      robot_msg.orientation = 0.0;  // invalid value
    }
    robot_msg.pixel_x = robot.pixel_x();
    robot_msg.pixel_y = robot.pixel_y();
    if (robot.has_height()) {
      robot_msg.height = robot.height() / 1000.0;
    } else {
      robot_msg.height = 0.0;  // invalid value
    }

    detection_frame_msg.robots_yellow.push_back(robot_msg);
  }

  // Parse blue robots（vision_componentと同じ構造）
  for (const auto & robot : ssl_detection.robots_blue()) {
    robocup_ssl_msgs::msg::DetectionRobot robot_msg;
    robot_msg.confidence = robot.confidence();
    if (robot.has_robot_id()) {
      robot_msg.robot_id = robot.robot_id();
    } else {
      robot_msg.robot_id = 100;  // invalid value
    }
    robot_msg.x = robot.x() / 1000.0;
    robot_msg.y = robot.y() / 1000.0;
    if (robot.has_orientation()) {
      robot_msg.orientation = robot.orientation();
    } else {
      robot_msg.orientation = 0.0;  // invalid value
    }
    robot_msg.pixel_x = robot.pixel_x();
    robot_msg.pixel_y = robot.pixel_y();
    if (robot.has_height()) {
      robot_msg.height = robot.height() / 1000.0;
    } else {
      robot_msg.height = 0.0;  // invalid value
    }

    detection_frame_msg.robots_blue.push_back(robot_msg);
  }

  return detection_frame_msg;
}

auto WorldModelDataProvider::reportError(const std::string & error_message) -> void
{
  RCLCPP_WARN(node.get_logger(), "WorldModelDataProvider error: %s", error_message.c_str());
}

}  // namespace crane
