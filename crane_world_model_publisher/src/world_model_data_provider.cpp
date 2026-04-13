// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/world_model_data_provider.hpp"

#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <crane_msg_wrappers/delay_monitor_wrapper.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <crane_visualization_interfaces/crane_visualizer_wrapper.hpp>
#include <filesystem>
#include <robocup_ssl_msgs/msg/robot_id.hpp>
#include <string>
#include <unordered_map>
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
  last_prediction_time_(node.get_clock()->now()),
  last_vision_recv_time_(node.get_clock()->now()),
  last_tracker_recv_time_(node.get_clock()->now()),
  has_tracked_frame_updated_(false)
{
  using std::chrono_literals::operator""ms;

  // VisionStreamProcessorの機能を統合：パラメータ設定
  node.declare_parameter("vision_address", config_.vision_address);
  node.declare_parameter("vision_port", config_.vision_port);
  node.declare_parameter("confidence_threshold", config_.confidence_threshold);
  // Tracker/legacy切替パラメータ
  node.declare_parameter("tracker_address", std::string("224.5.23.2"));
  node.declare_parameter("tracker_port", 10010);
  node.declare_parameter("use_udp_detection", false);
  node.declare_parameter("feedback_stale_timeout_ms", feedback_stale_timeout_ms_);

  config_.vision_address = node.get_parameter("vision_address").get_value<std::string>();
  config_.vision_port = node.get_parameter("vision_port").get_value<int>();
  config_.confidence_threshold = node.get_parameter("confidence_threshold").get_value<double>();
  use_udp_detection_ = node.get_parameter("use_udp_detection").get_value<bool>();
  feedback_stale_timeout_ms_ = node.get_parameter("feedback_stale_timeout_ms").get_value<int>();

  // Initialize UDP receivers

  // AsyncUdpReceiver初期化（Vision UDP）
  try {
    multicast_receiver_ = std::make_unique<crane::AsyncUdpReceiver>(
      asio_ctx_.io_context, config_.vision_address, config_.vision_port);
    multicast_receiver_->startReceive([this](const std::vector<char> & buf, size_t size) {
      if (size > 0) {
        std::lock_guard<std::mutex> lock(recv_mutex_);
        pending_vision_packets_.emplace_back(buf.data(), size);
      }
    });
    RCLCPP_INFO(
      node.get_logger(), "WorldModelDataProvider Vision設定: %s:%d", config_.vision_address.c_str(),
      config_.vision_port);
  } catch (const std::exception & ex) {
    reportError("Failed to initialize vision stream: " + std::string(ex.what()));
    RCLCPP_ERROR(node.get_logger(), "Vision initialization failed: %s", ex.what());
  }

  // AsyncUdpReceiver初期化（Tracker UDP）
  try {
    config_.tracker_address = node.get_parameter("tracker_address").get_value<std::string>();
    config_.tracker_port = node.get_parameter("tracker_port").get_value<int>();
    tracker_receiver_ = std::make_unique<crane::AsyncUdpReceiver>(
      asio_ctx_.io_context, config_.tracker_address, config_.tracker_port);
    tracker_receiver_->startReceive([this](const std::vector<char> & buf, size_t size) {
      if (size > 0) {
        std::lock_guard<std::mutex> lock(recv_mutex_);
        pending_tracker_packets_.emplace_back(buf.data(), size);
      }
    });
    RCLCPP_INFO(
      node.get_logger(), "WorldModelDataProvider Tracker設定: %s:%d",
      config_.tracker_address.c_str(), config_.tracker_port);
  } catch (const std::exception & ex) {
    reportError("Trackerの初期化に失敗しました: " + std::string(ex.what()));
    RCLCPP_ERROR(node.get_logger(), "Trackerの初期化に失敗しました: %s", ex.what());
  }

  asio_ctx_.start();

  // ロボット情報初期化
  for (int team = 0; team < 2; ++team) {
    robot_info_[team].resize(MAX_ROBOT_COUNT);
    error_tracker_[team].resize(MAX_ROBOT_COUNT);
    for (size_t i = 0; i < MAX_ROBOT_COUNT; ++i) {
      auto & robot = robot_info_[team][i];
      robot.id = static_cast<uint8_t>(i);
      robot.available_vision = false;
      robot.available_feedback = false;
      robot.available_tracker = false;
      // error tracker defaults are already zeroed by the struct's default members
    }
  }

  // ボール情報初期化
  ball_info_.detected = false;
  ball_info_.vision_detected = false;
  ball_info_.tracker_detected = false;
  ball_info_.state = crane_msgs::msg::BallInfo::STOPPED;

  // ボール状態の初期化
  vision_ball_state_.last_detect_time = node.get_clock()->now();
  tracker_ball_state_.last_detect_time = node.get_clock()->now();

  area_mask.min_corner() << -20., -10.;
  area_mask.max_corner() << 20., 10.;

  // フィールドジオメトリ設定ファイルの読み込み
  node.declare_parameter("field_geometry_config_path", "");
  std::string field_geometry_config_path =
    node.get_parameter("field_geometry_config_path").as_string();
  if (!field_geometry_config_path.empty()) {
    // ファイル名だけの場合はconfigディレクトリと結合
    std::string full_config_path = field_geometry_config_path;
    if (!std::filesystem::path(field_geometry_config_path).is_absolute()) {
      try {
        std::string package_share_dir =
          ament_index_cpp::get_package_share_directory("crane_world_model_publisher");
        full_config_path =
          std::filesystem::path(package_share_dir) / "config" / field_geometry_config_path;
      } catch (const std::exception & ex) {
        RCLCPP_WARN(
          node.get_logger(),
          "パッケージディレクトリの取得に失敗しました: %s 相対パスとして扱います", ex.what());
      }
    }

    if (loadFieldGeometryFromConfig(full_config_path)) {
      geometry_initialized = true;
      updateGeometryIfNeeded();
    }
  }

  udp_timer = node.create_wall_timer(10ms, std::bind(&WorldModelDataProvider::on_udp_timer, this));

  // 受信監視: 1秒周期でVision/Trackerの受信有無をチェックし、欠損時のみログ出力
  status_check_timer_ = node.create_wall_timer(1000ms, [this]() {
    auto now = this->node.get_clock()->now();

    if (multicast_receiver_ && use_udp_detection_) {
      if ((now - last_vision_recv_time_).seconds() > 1.0) {
        RCLCPP_WARN(
          this->node.get_logger(), "Vision受信が直近1秒間ありません (%s:%d)",
          config_.vision_address.c_str(), config_.vision_port);
      }
    }

    if (tracker_receiver_) {
      if ((now - last_tracker_recv_time_).seconds() > 1.0) {
        RCLCPP_WARN(
          this->node.get_logger(), "Tracker受信が直近1秒間ありません (%s:%d)",
          config_.tracker_address.c_str(), config_.tracker_port);
      }
    }
  });

  // /play_situationのトピック統計はsession_controllerで取得
  sub_play_situation = node.create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 1,
    [this](const crane_msgs::msg::PlaySituation msg) { latest_play_situation = msg; });

  sub_robot_feedback = node.create_subscription<crane_msgs::msg::RobotFeedbackArray>(
    "/robot_feedback", 1,
    [this](const crane_msgs::msg::RobotFeedbackArray::SharedPtr msg) { robot_feedback = *msg; });

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

  // 半面練習モード: session coordinator からの /practice_mode トピックを購読
  sub_practice_mode = node.create_subscription<crane_msgs::msg::PracticeMode>(
    "/practice_mode", 1,
    [this](const crane_msgs::msg::PracticeMode & msg) { latest_practice_mode = msg; });

  sub_referee = node.create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 1, [this](const robocup_ssl_msgs::msg::Referee & msg) {
      if (msg.yellow.name == game_data.team_name) {
        // YELLOW
        game_data.our_color = Color::YELLOW;
        game_data.their_color = Color::BLUE;
        our_team_color_ = TeamColor::YELLOW;
        game_data.our_goalie_id = msg.yellow.goalkeeper;
        game_data.their_goalie_id = msg.blue.goalkeeper;
        if (msg.yellow.has_field & msg.yellow.MAX_ALLOWED_BOTS_FIELD_SET) {
          game_data.our_max_allowed_bots = msg.yellow.max_allowed_bots;
        }
        if (msg.blue.has_field & msg.blue.MAX_ALLOWED_BOTS_FIELD_SET) {
          game_data.their_max_allowed_bots = msg.blue.max_allowed_bots;
        }
        if (msg.has_field & msg.BLUE_TEAM_ON_POSITIVE_HALF_FIELD_SET) {
          on_positive_half = (msg.blue_team_on_positive_half == false);
        }
      } else if (msg.blue.name == game_data.team_name) {
        // BLUE
        game_data.our_color = Color::BLUE;
        game_data.their_color = Color::YELLOW;
        our_team_color_ = TeamColor::BLUE;
        game_data.our_goalie_id = msg.blue.goalkeeper;
        game_data.their_goalie_id = msg.yellow.goalkeeper;
        if (msg.blue.has_field & msg.blue.MAX_ALLOWED_BOTS_FIELD_SET) {
          game_data.our_max_allowed_bots = msg.blue.max_allowed_bots;
        }
        if (msg.yellow.has_field & msg.yellow.MAX_ALLOWED_BOTS_FIELD_SET) {
          game_data.their_max_allowed_bots = msg.yellow.max_allowed_bots;
        }
        if (msg.has_field & msg.BLUE_TEAM_ON_POSITIVE_HALF_FIELD_SET) {
          on_positive_half = (msg.blue_team_on_positive_half == true);
        }
      } else {
        std::stringstream what;
        what << "味方チーム名, " << std::string(game_data.team_name)
             << " がレフェリー信号の中に見当たりません。 ";
        what << "青チーム: " << std::string(msg.blue.name)
             << ", 黄色チーム: " << std::string(msg.yellow.name);
        reportError(what.str());
      }

      if (referee_visualization_callback_) {
        referee_visualization_callback_(msg, game_data.field_w, game_data.field_h);
      }
      CraneVisualizerBuffer::publish();
    });

  // direct UDP from Tracker; no ROS topic subscription
}

WorldModelDataProvider::~WorldModelDataProvider() = default;

auto WorldModelDataProvider::on_udp_timer() -> void
{
  // asioスレッドからのパケットを取り出す（最小限のロック）
  std::vector<std::string> vision_packets, tracker_packets;
  {
    std::lock_guard<std::mutex> lock(recv_mutex_);
    vision_packets.swap(pending_vision_packets_);
    tracker_packets.swap(pending_tracker_packets_);
  }

  // Visionパケット処理（ROS2スレッドから安全に実行）
  for (const auto & raw : vision_packets) {
    try {
      robocup_ssl::SSL_WrapperPacket packet;
      if (packet.ParseFromString(raw)) {
        if (packet.has_detection()) {
          // Vision ボール生データは常に更新（isBallTrulyLostFromDribblerのクロスリファレンス用）
          if (!packet.detection().balls().empty()) {
            const auto & balls = packet.detection().balls();
            size_t idx = 0;
            if (latest_practice_mode.enabled && balls.size() > 1) {
              idx = selectClosestBallToOurGoal(balls, [](const auto & b) {
                return std::make_pair(b.x() / 1000.0, b.y() / 1000.0);
              });
            }
            updateVisionBallState(
              balls.at(static_cast<int>(idx)),
              static_cast<uint32_t>(packet.detection().camera_id()));
            integrateBallInfo();
          }
          if (use_udp_detection_) {
            processDetectionFrame(packet.detection());
            has_vision_updated_ = true;
            last_vision_recv_time_ = node.get_clock()->now();
          }
        }
        if (packet.has_geometry()) {
          processGeometryData(packet.geometry());
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(node.get_logger(), "Visionパケットパースエラー: %s", ex.what());
    }
  }

  // Trackerパケット処理（ROS2スレッドから安全に実行）
  for (const auto & raw : tracker_packets) {
    try {
      robocup_ssl::TrackerWrapperPacket wrapper_packet;
      if (wrapper_packet.ParseFromString(raw) && wrapper_packet.has_tracked_frame()) {
        auto tracked_frame_msg = parseTrackedFrameFromWrapper(wrapper_packet);
        latest_tracked_frame = tracked_frame_msg;
        has_tracked_frame_updated_ = true;
        last_tracker_recv_time_ = node.get_clock()->now();
        processTrackedFrame(tracked_frame_msg);
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(node.get_logger(), "Trackerパケットパースエラー: %s", ex.what());
    }
  }

  // 設定ファイルで初期化済みでも、Vision geometry受信後の更新を反映するため毎周期評価する
  updateGeometryIfNeeded();
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
    if (geometry_initialized) {
      RCLCPP_INFO(
        node.get_logger(),
        "フィールド情報更新: field=%.3fx%.3f, goal=%.3fx%.3f, penalty_area=%.3fx%.3f",
        game_data.field_w, game_data.field_h, game_data.goal_w, game_data.goal_h,
        game_data.penalty_area_w, game_data.penalty_area_h);
    } else {
      RCLCPP_INFO(
        node.get_logger(),
        "フィールド情報初期化完了: field=%.3fx%.3f, goal=%.3fx%.3f, penalty_area=%.3fx%.3f "
        "→ world_model発行が開始されます",
        game_data.field_w, game_data.field_h, game_data.goal_w, game_data.goal_h,
        game_data.penalty_area_w, game_data.penalty_area_h);
    }
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

  // 半面練習モード設定（session coordinator から /practice_mode トピック経由）
  msg.practice_mode = latest_practice_mode;

  // Get ball data directly from local data
  if (has_vision_updated_ || has_tracked_frame_updated_) {
    msg.ball_info = ball_info_;
  } else {
    // Use default ball info when vision is not available
    msg.ball_info = crane_msgs::msg::BallInfo{};
    RCLCPP_WARN_THROTTLE(
      node.get_logger(), *node.get_clock(), 5000, "新しいボールデータがありません");
  }

  auto current_time = node.get_clock()->now();

  // robot_feedback を ID → ポインタのマップに変換して O(N×M) → O(N) に削減
  std::unordered_map<uint8_t, const crane_msgs::msg::RobotFeedback *> feedback_map;
  for (const auto & fb : robot_feedback.feedback) {
    feedback_map[static_cast<uint8_t>(fb.robot_id)] = &fb;
  }

  std::vector<crane_msgs::msg::RobotInfo> team_0_robots;
  std::vector<crane_msgs::msg::RobotInfo> team_1_robots;

  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    auto vision_robots = robot_info_[team_idx];

    // robot_feedbackデータを統合
    for (auto & robot : vision_robots) {
      if (auto it = feedback_map.find(robot.id); it != feedback_map.end()) {
        const auto & feedback = *it->second;
        const auto feedback_age_ms =
          (current_time - rclcpp::Time(feedback.received_stamp)).seconds() * 1000.0;
        if (feedback_age_ms > static_cast<double>(feedback_stale_timeout_ms_)) {
          continue;
        }

        // feedbackデータを含むRobotInfoを作成
        crane_msgs::msg::RobotInfo feedback_robot;
        feedback_robot.id = feedback.robot_id;
        feedback_robot.available_feedback = true;
        feedback_robot.ball_sensor = feedback.ball_sensor;
        feedback_robot.last_ball_sensor_stamp = feedback.received_stamp;
        feedback_robot.last_feedback_detection_stamp = feedback.received_stamp;

        // エラー情報を転送 + 継続時間の算出
        bool has_err = (feedback.error_id != 0 || feedback.error_info != 0);
        feedback_robot.has_error = has_err;
        feedback_robot.error_id = feedback.error_id;
        feedback_robot.error_info = feedback.error_info;
        feedback_robot.error_value = feedback.error_value;
        feedback_robot.last_error_stamp = feedback.received_stamp;

        auto & err_tracker = error_tracker_[team_idx][robot.id];
        if (has_err) {
          if (!err_tracker.active) {
            // Any error started now
            err_tracker.active = true;
            err_tracker.start = rclcpp::Time(feedback.received_stamp);
          }
          // 稼働中のエラーとして種類(id/info)のみ更新（開始時刻はリセットしない）
          err_tracker.id = feedback.error_id;
          err_tracker.info = feedback.error_info;
          double duration = (current_time - err_tracker.start).seconds();
          if (duration < 0.0) duration = 0.0;
          feedback_robot.error_duration_sec = static_cast<float>(duration);
        } else {
          err_tracker.active = false;
          err_tracker.id = 0;
          err_tracker.info = 0;
          feedback_robot.error_duration_sec = 0.0f;
        }

        // モーター温度情報を転送
        feedback_robot.motor_temperatures = feedback.temperatures;

        // visionデータとfeedbackデータを統合
        robot = mergeRobotInfo(robot, feedback_robot);
      }
    }

    if (team_idx == 0) {
      team_0_robots = std::move(vision_robots);
    } else {
      team_1_robots = std::move(vision_robots);
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
    "チーム割当: 味方カラー=%s, 味方ロボット数=%zu, 敵ロボット数=%zu",
    (our_team_color_ == TeamColor::BLUE) ? "青" : "黄", msg.robot_info_ours.size(),
    msg.robot_info_theirs.size());

  msg.field_info.x = game_data.field_w;
  msg.field_info.y = game_data.field_h;

  msg.penalty_area_size.x = game_data.penalty_area_h;
  msg.penalty_area_size.y = game_data.penalty_area_w;

  msg.goal_size.x = game_data.goal_h;
  msg.goal_size.y = game_data.goal_w;

  // Vision遅延情報をDelayCheckpointに追加
  if (last_t_capture_ > 0.0 && last_t_sent_ > 0.0) {
    std::string vision_delay_info =
      DelayMonitorWrapper::formatVisionDelayInfo(last_t_capture_, last_t_sent_, current_time);

    DelayMonitorWrapper::addDelayCheckpoint(
      msg.delay_checkpoints, "vision_timestamps", vision_delay_info);
  }

  if (game_data.field_w <= 0.0 || game_data.field_h <= 0.0) {
    static rclcpp::Time last_warning_time = node.get_clock()->now();
    // Warn every 5 seconds to avoid spam
    if ((current_time - last_warning_time).seconds() > 5.0) {
      RCLCPP_WARN(
        node.get_logger(),
        "不正なフィールド情報です: field=%.3fx%.3f, goal=%.3fx%.3f, "
        "penalty_area=%.3fx%.3f",
        game_data.field_w, game_data.field_h, game_data.goal_w, game_data.goal_h,
        game_data.penalty_area_w, game_data.penalty_area_h);
      last_warning_time = current_time;
    }
  }

  msg.header.stamp = current_time;
  return msg;
}

auto WorldModelDataProvider::setVisualizationCallbacks(
  std::function<void(const robocup_ssl::SSL_GeometryData &, bool)> geometry_callback,
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
  merged.available_feedback = feedback_robot.available_feedback;
  merged.ball_sensor = feedback_robot.ball_sensor;
  merged.last_ball_sensor_stamp = feedback_robot.last_ball_sensor_stamp;
  merged.last_feedback_detection_stamp = feedback_robot.last_feedback_detection_stamp;

  // Merge error information
  merged.has_error = feedback_robot.has_error;
  merged.error_id = feedback_robot.error_id;
  merged.error_info = feedback_robot.error_info;
  merged.error_value = feedback_robot.error_value;
  merged.last_error_stamp = feedback_robot.last_error_stamp;
  merged.error_duration_sec = feedback_robot.error_duration_sec;

  return merged;
}

auto WorldModelDataProvider::processDetectionFrame(
  const robocup_ssl::SSL_DetectionFrame & detection) -> bool
{
  // 最新のSSL_DetectionFrameを保存（detection_frame生成用）
  latest_ssl_detection_frame_ = detection;
  has_latest_detection_frame_ = true;

  // タイムスタンプ更新
  last_t_capture_ = detection.t_capture();
  last_t_sent_ = detection.t_sent();

  // ボール検出処理
  if (!detection.balls().empty()) {
    const auto & balls = detection.balls();
    size_t idx = 0;
    if (latest_practice_mode.enabled && balls.size() > 1) {
      idx = selectClosestBallToOurGoal(
        balls, [](const auto & b) { return std::make_pair(b.x() / 1000.0, b.y() / 1000.0); });
    }
    updateVisionBallState(
      balls.at(static_cast<int>(idx)), static_cast<uint32_t>(detection.camera_id()));
  }

  // Vision/Tracker状態を統合してball_info_を更新
  integrateBallInfo();

  // ロボット検出は現在TrackedFrameで処理される
  // Vision detection frameは主にgeometry情報のために保持

  return true;
}

auto WorldModelDataProvider::processGeometryData(const robocup_ssl::SSL_GeometryData & geometry)
  -> bool
{
  convertFieldGeometry(geometry);

  // カメラ位置をキャッシュ（occlusion shadow 推定に使用）
  for (int i = 0; i < geometry.calib_size(); ++i) {
    const auto & calib = geometry.calib(i);
    if (
      calib.has_derived_camera_world_tx() && calib.has_derived_camera_world_ty() &&
      calib.has_derived_camera_world_tz()) {
      // mm → m 変換
      camera_positions_[static_cast<uint32_t>(calib.camera_id())] = Eigen::Vector3d(
        calib.derived_camera_world_tx() / 1000.0, calib.derived_camera_world_ty() / 1000.0,
        calib.derived_camera_world_tz() / 1000.0);
    }
  }

  if (geometry_visualization_callback_) {
    geometry_visualization_callback_(geometry, false);  // half_court_mode = false
  }

  return true;
}

auto WorldModelDataProvider::estimateFallbackBall(const rclcpp::Time & now) -> void
{
  constexpr double kBallSensorTimeoutSec = 0.2;
  constexpr double kLastKnownTimeoutSec = 2.0;
  // ロボット半径(0.09m) + ボール半径(0.0215m) の合計でオクルージョン判定
  constexpr double kOcclusionShadowRadius = 0.09 + 0.0215;

  auto setFallback = [&](
                       uint8_t source, const Eigen::Vector2d & pos, float conf, uint32_t occluder,
                       const rclcpp::Time & stamp) {
    ball_info_.fallback_available = true;
    ball_info_.fallback_source = source;
    ball_info_.fallback_position.x = pos.x();
    ball_info_.fallback_position.y = pos.y();
    ball_info_.fallback_position.z = 0.0;
    ball_info_.fallback_confidence = conf;
    ball_info_.fallback_occluder_robot_id = occluder;
    ball_info_.fallback_stamp = stamp;
  };

  auto clearFallback = [&]() {
    ball_info_.fallback_available = false;
    ball_info_.fallback_source = crane_msgs::msg::BallInfo::FALLBACK_NONE;
    ball_info_.fallback_confidence = 0.0f;
    ball_info_.fallback_occluder_robot_id = 0;
    ball_info_.fallback_stamp = now;
  };

  // 優先度 1: 物理センサ反応は最も信頼度が高い
  if (ball_sensor_hint_) {
    if ((now - ball_sensor_hint_->stamp).seconds() <= kBallSensorTimeoutSec) {
      setFallback(
        crane_msgs::msg::BallInfo::FALLBACK_BALL_SENSOR, ball_sensor_hint_->position, 0.9f,
        ball_sensor_hint_->robot_id, ball_sensor_hint_->stamp);
      return;
    }
    ball_sensor_hint_ = std::nullopt;
  }

  if (!last_known_ball_valid_) {
    clearFallback();
    return;
  }

  double last_known_age = (now - last_known_ball_stamp_).seconds();

  // 優先度 2: カメラの視線がロボットで遮られている場合、ボールは最終観測位置にある可能性が高い
  if (last_known_ball_camera_id_ && last_known_age <= kLastKnownTimeoutSec) {
    auto cam_it = camera_positions_.find(*last_known_ball_camera_id_);
    if (cam_it != camera_positions_.end()) {
      const Eigen::Vector3d & cam3d = cam_it->second;
      Eigen::Vector2d cam_xy(cam3d.x(), cam3d.y());
      Eigen::Vector2d ball_xy(last_known_ball_position_.x(), last_known_ball_position_.y());
      Eigen::Vector2d ray = ball_xy - cam_xy;
      double ray_len = ray.norm();

      if (ray_len > 1e-6) {
        Eigen::Vector2d ray_dir = ray / ray_len;
        for (int team = 0; team < 2; ++team) {
          for (const auto & robot_info : robot_info_[team]) {
            if (!robot_info.available_vision && !robot_info.available_tracker) continue;
            Eigen::Vector2d robot_xy(robot_info.pose.x, robot_info.pose.y);
            double t = (robot_xy - cam_xy).dot(ray_dir) / ray_len;
            if (t <= 0.0 || t >= 1.0) continue;
            double dist_to_ray = (robot_xy - (cam_xy + t * ray)).norm();
            if (dist_to_ray < kOcclusionShadowRadius) {
              float conf =
                static_cast<float>(std::max(0.3, 1.0 - last_known_age / kLastKnownTimeoutSec));
              setFallback(
                crane_msgs::msg::BallInfo::FALLBACK_OCCLUSION_SHADOW, ball_xy, conf,
                static_cast<uint32_t>(robot_info.id), last_known_ball_stamp_);
              return;
            }
          }
        }
      }
    }
  }

  // 優先度 3: 最終観測位置（信頼度は時間とともに線形減衰）
  if (last_known_age <= kLastKnownTimeoutSec) {
    float conf = static_cast<float>(std::max(0.0, 1.0 - last_known_age / kLastKnownTimeoutSec));
    setFallback(
      crane_msgs::msg::BallInfo::FALLBACK_LAST_KNOWN,
      Eigen::Vector2d(last_known_ball_position_.x(), last_known_ball_position_.y()), conf, 0,
      last_known_ball_stamp_);
    return;
  }

  clearFallback();
}

auto WorldModelDataProvider::updateVisionBallState(
  const robocup_ssl::SSL_DetectionBall & ssl_ball, uint32_t camera_id) -> void
{
  // 座標変換 (mm -> m)
  double x = ssl_ball.x() / 1000.0;
  double y = ssl_ball.y() / 1000.0;
  double z = ssl_ball.has_z() ? ssl_ball.z() / 1000.0 : 0.0;

  // NaN/Inf チェック: 無効パケットは無視してworld modelを汚染させない
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    RCLCPP_WARN(
      node.get_logger(),
      "Vision ball position contains NaN/Inf: (%.3f, %.3f, %.3f) - skipping packet", x, y, z);
    return;
  }

  auto now = node.get_clock()->now();

  // Vision状態を更新
  vision_ball_state_.position = Eigen::Vector3d(x, y, z);
  vision_ball_state_.last_detect_time = now;
  vision_ball_state_.detected = true;
  vision_ball_state_.raw_position.x = x;
  vision_ball_state_.raw_position.y = y;
  vision_ball_state_.raw_position.z = z;

  // フォールバック用: 最終観測情報を記録
  last_known_ball_position_ = Eigen::Vector3d(x, y, z);
  last_known_ball_stamp_ = now;
  last_known_ball_valid_ = true;
  last_known_ball_camera_id_ = camera_id;

  RCLCPP_DEBUG(node.get_logger(), "Vision ball updated at (%.3f, %.3f, %.3f)", x, y, z);
}

auto WorldModelDataProvider::convertFieldGeometry(
  const robocup_ssl::SSL_GeometryData & ssl_geometry) -> void
{
  if (!ssl_geometry.has_field()) {
    RCLCPP_WARN(
      node.get_logger(), "GeometryデータにFieldが含まれていません（has_field() = false）");
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

auto WorldModelDataProvider::loadFieldGeometryFromConfig(const std::string & config_path) -> bool
{
  if (config_path.empty()) {
    return false;
  }

  try {
    YAML::Node config = YAML::LoadFile(config_path);
    if (!config["field_geometry"]) {
      RCLCPP_WARN(
        node.get_logger(), "設定ファイルに'field_geometry'セクションが見つかりません: %s",
        config_path.c_str());
      return false;
    }

    auto geometry = config["field_geometry"];

    field_geometry_.field_width = geometry["field_length"].as<double>();
    field_geometry_.field_height = geometry["field_width"].as<double>();
    field_geometry_.goal_width = geometry["goal_width"].as<double>();
    field_geometry_.goal_height = geometry["goal_depth"].as<double>();
    field_geometry_.penalty_area_height = geometry["penalty_area_depth"].as<double>();
    field_geometry_.penalty_area_width = geometry["penalty_area_width"].as<double>();
    field_geometry_.center_circle_radius = geometry["center_circle_radius"].as<double>(0.5);
    field_geometry_.is_valid = true;

    RCLCPP_INFO(
      node.get_logger(),
      "設定ファイルからフィールド情報を読み込みました: field=%.3fx%.3f, goal=%.3fx%.3f, "
      "penalty_area=%.3fx%.3f",
      field_geometry_.field_width, field_geometry_.field_height, field_geometry_.goal_width,
      field_geometry_.goal_height, field_geometry_.penalty_area_width,
      field_geometry_.penalty_area_height);
    return true;
  } catch (const YAML::Exception & ex) {
    RCLCPP_WARN(
      node.get_logger(), "フィールド設定ファイルのYAMLパースエラー: %s (file: %s)", ex.what(),
      config_path.c_str());
    return false;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      node.get_logger(), "フィールド設定ファイルの読み込みに失敗: %s (file: %s)", ex.what(),
      config_path.c_str());
    return false;
  }
}

auto WorldModelDataProvider::reportError(const std::string & error_message) -> void
{
  RCLCPP_WARN_THROTTLE(
    node.get_logger(), *node.get_clock(), 1000, "WorldModelDataProvider error: %s",
    error_message.c_str());
}

auto WorldModelDataProvider::processTrackedFrame(
  const robocup_ssl_msgs::msg::TrackedFrame & tracked_frame) -> void
{
  auto now = node.get_clock()->now();

  // ボール情報の処理
  if (!tracked_frame.balls.empty()) {
    size_t idx = 0;
    if (latest_practice_mode.enabled && tracked_frame.balls.size() > 1) {
      idx = selectClosestBallToOurGoal(tracked_frame.balls, [](const auto & b) {
        return std::make_pair(static_cast<double>(b.pos.x), static_cast<double>(b.pos.y));
      });
    }
    updateTrackerBallState(tracked_frame.balls[idx]);
  }

  // Vision/Tracker状態を統合してball_info_を更新
  integrateBallInfo();

  // ロボット情報の処理
  // 全チーム・全ロボットIDをリセット
  for (int team_idx = 0; team_idx < 2; ++team_idx) {
    for (auto & robot : robot_info_[team_idx]) {
      robot.available_vision = false;
    }
  }

  // TrackedRobotから情報を変換
  for (const auto & tracked_robot : tracked_frame.robots) {
    uint8_t robot_id = static_cast<uint8_t>(tracked_robot.robot_id.id);
    if (robot_id >= MAX_ROBOT_COUNT) continue;
    int team_index = (tracked_robot.robot_id.team.value == robocup_ssl_msgs::msg::Team::YELLOW)
                       ? static_cast<int>(Color::YELLOW)
                       : static_cast<int>(Color::BLUE);

    auto & robot = robot_info_[team_index][robot_id];
    robot = convertTrackedRobot(tracked_robot, team_index);
    robot.vision.stamp = now;
  }
}

auto WorldModelDataProvider::updateTrackerBallState(
  const robocup_ssl_msgs::msg::TrackedBall & tracked_ball) -> void
{
  auto now = node.get_clock()->now();

  // NaN/Inf チェック: 位置が無効なパケットは無視してworld modelを汚染させない
  if (
    !std::isfinite(tracked_ball.pos.x) || !std::isfinite(tracked_ball.pos.y) ||
    !std::isfinite(tracked_ball.pos.z)) {
    RCLCPP_WARN(
      node.get_logger(),
      "Tracker ball position contains NaN/Inf: (%.3f, %.3f, %.3f) - skipping packet",
      tracked_ball.pos.x, tracked_ball.pos.y, tracked_ball.pos.z);
    return;
  }

  // Tracker状態を更新
  tracker_ball_state_.position =
    Eigen::Vector3d(tracked_ball.pos.x, tracked_ball.pos.y, tracked_ball.pos.z);

  // 速度情報（オプション）
  if (tracked_ball.has_field & tracked_ball.VEL_FIELD_SET) {
    const auto & vel = tracked_ball.vel;
    if (!std::isfinite(vel.x) || !std::isfinite(vel.y) || !std::isfinite(vel.z)) {
      RCLCPP_WARN(
        node.get_logger(),
        "Tracker ball velocity contains NaN/Inf: (%.3f, %.3f, %.3f) - using zero", vel.x, vel.y,
        vel.z);
      tracker_ball_state_.velocity = Eigen::Vector3d::Zero();
    } else {
      tracker_ball_state_.velocity = Eigen::Vector3d(vel.x, vel.y, vel.z);
    }
  } else {
    tracker_ball_state_.velocity = Eigen::Vector3d::Zero();
  }

  tracker_ball_state_.last_detect_time = now;
  tracker_ball_state_.detected = true;

  RCLCPP_DEBUG(
    node.get_logger(),
    "Tracker ball updated at (%.3f, %.3f, %.3f) with velocity (%.3f, %.3f, %.3f)",
    tracked_ball.pos.x, tracked_ball.pos.y, tracked_ball.pos.z, tracker_ball_state_.velocity.x(),
    tracker_ball_state_.velocity.y(), tracker_ball_state_.velocity.z());
}

auto WorldModelDataProvider::integrateBallInfo() -> void
{
  auto now = node.get_clock()->now();
  constexpr double TIMEOUT_SEC = 0.1;

  // タイムアウトチェック
  if ((now - vision_ball_state_.last_detect_time).seconds() > TIMEOUT_SEC) {
    vision_ball_state_.detected = false;
  }
  if ((now - tracker_ball_state_.last_detect_time).seconds() > TIMEOUT_SEC) {
    tracker_ball_state_.detected = false;
  }

  // 統合フラグ設定
  ball_info_.vision_detected = vision_ball_state_.detected;
  ball_info_.tracker_detected = tracker_ball_state_.detected;
  ball_info_.detected = vision_ball_state_.detected || tracker_ball_state_.detected;

  // 位置・速度の決定（Tracker優先）
  if (tracker_ball_state_.detected) {
    ball_info_.position.x = tracker_ball_state_.position.x();
    ball_info_.position.y = tracker_ball_state_.position.y();
    ball_info_.position.z = tracker_ball_state_.position.z();
    ball_info_.velocity.x = tracker_ball_state_.velocity.x();
    ball_info_.velocity.y = tracker_ball_state_.velocity.y();
    ball_info_.velocity.z = tracker_ball_state_.velocity.z();
    ball_info_.velocity_norm = tracker_ball_state_.velocity.norm();
  } else if (vision_ball_state_.detected) {
    ball_info_.position.x = vision_ball_state_.position.x();
    ball_info_.position.y = vision_ball_state_.position.y();
    ball_info_.position.z = vision_ball_state_.position.z();
    // Visionからは速度計算しない（Trackerがない場合は速度0）
    ball_info_.velocity.x = 0.0;
    ball_info_.velocity.y = 0.0;
    ball_info_.velocity.z = 0.0;
    ball_info_.velocity_norm = 0.0;
  } else {
    // 両方未検出 - 速度のみリセット
    ball_info_.velocity.x = 0.0;
    ball_info_.velocity.y = 0.0;
    ball_info_.velocity.z = 0.0;
    ball_info_.velocity_norm = 0.0;
  }

  // Vision情報の更新（常にVisionの生データを保持）
  if (vision_ball_state_.detected) {
    ball_info_.vision.stamp = vision_ball_state_.last_detect_time;
    ball_info_.vision.pos = vision_ball_state_.raw_position;
  }

  // Tracker情報の更新（常にTrackerの生データを保持）
  if (tracker_ball_state_.detected) {
    ball_info_.tracker.stamp = tracker_ball_state_.last_detect_time;
    ball_info_.tracker.pos.x = tracker_ball_state_.position.x();
    ball_info_.tracker.pos.y = tracker_ball_state_.position.y();
    ball_info_.tracker.pos.z = tracker_ball_state_.position.z();
  }

  // Tracker はカメラ統合済みなので特定カメラに帰属しない → occlusion 推定には使えない
  if (tracker_ball_state_.detected) {
    last_known_ball_position_ = tracker_ball_state_.position;
    last_known_ball_stamp_ = now;
    last_known_ball_valid_ = true;
    last_known_ball_camera_id_ = std::nullopt;
  }

  // ボール状態の決定
  if (ball_info_.velocity_norm < 0.1) {
    ball_info_.state = crane_msgs::msg::BallInfo::STOPPED;
  } else {
    ball_info_.state = crane_msgs::msg::BallInfo::ROLLING;
  }

  if (!ball_info_.detected) {
    estimateFallbackBall(now);
  } else {
    ball_info_.fallback_available = false;
    ball_info_.fallback_source = crane_msgs::msg::BallInfo::FALLBACK_NONE;
    ball_info_.fallback_confidence = 0.0f;
    ball_info_.fallback_stamp = now;
    ball_info_.fallback_occluder_robot_id = 0;
  }
}

auto WorldModelDataProvider::convertTrackedRobot(
  const robocup_ssl_msgs::msg::TrackedRobot & tracked_robot, [[maybe_unused]] int team_index)
  -> crane_msgs::msg::RobotInfo
{
  crane_msgs::msg::RobotInfo robot_info;
  auto now = node.get_clock()->now();

  robot_info.id = static_cast<uint8_t>(tracked_robot.robot_id.id);

  // 検出フラグ（NaN位置のロボットは利用不可として扱う）
  const bool valid_position =
    std::isfinite(tracked_robot.pos.x) && std::isfinite(tracked_robot.pos.y);
  if (!valid_position) {
    RCLCPP_WARN_THROTTLE(
      node.get_logger(), *node.get_clock(), 1000,
      "Robot id=%d position contains NaN/Inf: (%.3f, %.3f) - marking unavailable",
      tracked_robot.robot_id.id, tracked_robot.pos.x, tracked_robot.pos.y);
  }

  // 位置・姿勢情報（NaN/Infは0にクランプ - available_*フラグで無効として扱われる）
  robot_info.pose.x = valid_position ? tracked_robot.pos.x : 0.0;
  robot_info.pose.y = valid_position ? tracked_robot.pos.y : 0.0;
  robot_info.pose.theta = crane::normalizeAngle(tracked_robot.orientation);

  // 速度情報（オプション）
  if (tracked_robot.has_field & tracked_robot.VEL_FIELD_SET) {
    const auto & vel = tracked_robot.vel;
    if (!std::isfinite(vel.x) || !std::isfinite(vel.y)) {
      RCLCPP_WARN_THROTTLE(
        node.get_logger(), *node.get_clock(), 1000,
        "Robot id=%d velocity contains NaN/Inf: (%.3f, %.3f) - using zero",
        tracked_robot.robot_id.id, vel.x, vel.y);
      robot_info.velocity.x = 0.0;
      robot_info.velocity.y = 0.0;
      robot_info.velocity_norm = 0.0;
    } else {
      robot_info.velocity.x = vel.x;
      robot_info.velocity.y = vel.y;
      robot_info.velocity_norm = std::hypot(vel.x, vel.y);
    }
  } else {
    robot_info.velocity.x = 0.0;
    robot_info.velocity.y = 0.0;
    robot_info.velocity_norm = 0.0;
  }

  // Vision情報（Trackerは内部でVisionを統合しているため、Vision情報としても扱う）
  robot_info.vision.stamp = now;
  robot_info.vision.pose.x = robot_info.pose.x;
  robot_info.vision.pose.y = robot_info.pose.y;
  robot_info.vision.pose.theta = tracked_robot.orientation;
  robot_info.available_vision = valid_position;
  robot_info.available_feedback = false;
  robot_info.available_tracker = valid_position;

  // タイムスタンプ設定
  robot_info.last_tracker_detection_stamp = now;

  return robot_info;
}

auto WorldModelDataProvider::parseTrackedFrameFromWrapper(
  const robocup_ssl::TrackerWrapperPacket & wrapper_packet) -> robocup_ssl_msgs::msg::TrackedFrame
{
  robocup_ssl_msgs::msg::TrackedFrame tracked_frame_msg;
  const auto & tracked_frame = wrapper_packet.tracked_frame();

  tracked_frame_msg.frame_number = tracked_frame.frame_number();
  tracked_frame_msg.timestamp = tracked_frame.timestamp();

  for (const auto & ball : tracked_frame.balls()) {
    robocup_ssl_msgs::msg::TrackedBall ball_msg;

    // Position (required)
    ball_msg.pos.x = ball.pos().x();
    ball_msg.pos.y = ball.pos().y();
    ball_msg.pos.z = ball.pos().z();
    ball_msg.has_field |= ball_msg.POS_FIELD_SET;

    // Velocity (optional)
    if (ball.has_vel()) {
      ball_msg.vel.x = ball.vel().x();
      ball_msg.vel.y = ball.vel().y();
      ball_msg.vel.z = ball.vel().z();
      ball_msg.has_field |= ball_msg.VEL_FIELD_SET;
    }

    // Visibility (optional)
    if (ball.has_visibility()) {
      ball_msg.visibility = ball.visibility();
      ball_msg.has_field |= ball_msg.VISIBILITY_FIELD_SET;
    }

    tracked_frame_msg.balls.push_back(ball_msg);
  }

  for (const auto & robot : tracked_frame.robots()) {
    robocup_ssl_msgs::msg::TrackedRobot robot_msg;

    // Robot ID (required)
    robot_msg.robot_id.id = robot.robot_id().id();
    robot_msg.robot_id.team.value = static_cast<int32_t>(robot.robot_id().team());
    robot_msg.robot_id.has_field |=
      robot_msg.robot_id.ID_FIELD_SET | robot_msg.robot_id.TEAM_FIELD_SET;

    // Position and orientation (required)
    robot_msg.pos.x = robot.pos().x();
    robot_msg.pos.y = robot.pos().y();
    robot_msg.has_field |= robot_msg.POS_FIELD_SET;
    robot_msg.orientation = robot.orientation();
    robot_msg.has_field |= robot_msg.ORIENTATION_FIELD_SET;

    // Velocity (optional)
    if (robot.has_vel()) {
      robot_msg.vel.x = robot.vel().x();
      robot_msg.vel.y = robot.vel().y();
      robot_msg.has_field |= robot_msg.VEL_FIELD_SET;
    }

    // Angular velocity (optional)
    if (robot.has_vel_angular()) {
      robot_msg.vel_angular = robot.vel_angular();
      robot_msg.has_field |= robot_msg.VEL_ANGULAR_FIELD_SET;
    }

    // Visibility (optional)
    if (robot.has_visibility()) {
      robot_msg.visibility = robot.visibility();
      robot_msg.has_field |= robot_msg.VISIBILITY_FIELD_SET;
    }

    tracked_frame_msg.robots.push_back(robot_msg);
  }

  if (tracked_frame.has_kicked_ball()) {
    const auto & kicked_ball = tracked_frame.kicked_ball();

    // Position (required)
    tracked_frame_msg.kicked_ball.pos.x = kicked_ball.pos().x();
    tracked_frame_msg.kicked_ball.pos.y = kicked_ball.pos().y();
    tracked_frame_msg.kicked_ball.has_field |= tracked_frame_msg.kicked_ball.POS_FIELD_SET;

    // Initial velocity (required)
    tracked_frame_msg.kicked_ball.vel.x = kicked_ball.vel().x();
    tracked_frame_msg.kicked_ball.vel.y = kicked_ball.vel().y();
    tracked_frame_msg.kicked_ball.vel.z = kicked_ball.vel().z();
    tracked_frame_msg.kicked_ball.has_field |= tracked_frame_msg.kicked_ball.VEL_FIELD_SET;

    // Start timestamp (required)
    tracked_frame_msg.kicked_ball.start_timestamp = kicked_ball.start_timestamp();
    tracked_frame_msg.kicked_ball.has_field |=
      tracked_frame_msg.kicked_ball.START_TIMESTAMP_FIELD_SET;

    // Stop timestamp (optional)
    if (kicked_ball.has_stop_timestamp()) {
      tracked_frame_msg.kicked_ball.stop_timestamp = kicked_ball.stop_timestamp();
      tracked_frame_msg.kicked_ball.has_field |=
        tracked_frame_msg.kicked_ball.STOP_TIMESTAMP_FIELD_SET;
    }

    // Stop position (optional)
    if (kicked_ball.has_stop_pos()) {
      tracked_frame_msg.kicked_ball.stop_pos.x = kicked_ball.stop_pos().x();
      tracked_frame_msg.kicked_ball.stop_pos.y = kicked_ball.stop_pos().y();
      tracked_frame_msg.kicked_ball.has_field |= tracked_frame_msg.kicked_ball.STOP_POS_FIELD_SET;
    }

    // Robot ID that kicked the ball (optional)
    if (kicked_ball.has_robot_id()) {
      tracked_frame_msg.kicked_ball.robot_id.id = kicked_ball.robot_id().id();
      tracked_frame_msg.kicked_ball.robot_id.team.value =
        static_cast<int32_t>(kicked_ball.robot_id().team());
      tracked_frame_msg.kicked_ball.robot_id.has_field |=
        tracked_frame_msg.kicked_ball.robot_id.ID_FIELD_SET |
        tracked_frame_msg.kicked_ball.robot_id.TEAM_FIELD_SET;
      tracked_frame_msg.kicked_ball.has_field |= tracked_frame_msg.kicked_ball.ROBOT_ID_FIELD_SET;
    }

    tracked_frame_msg.has_field |= tracked_frame_msg.KICKED_BALL_FIELD_SET;
  }

  for (const auto & capability : tracked_frame.capabilities()) {
    robocup_ssl_msgs::msg::Capability cap_msg;
    cap_msg.value = static_cast<int32_t>(capability);
    tracked_frame_msg.capabilities.push_back(cap_msg);
  }

  return tracked_frame_msg;
}
}  // namespace crane
