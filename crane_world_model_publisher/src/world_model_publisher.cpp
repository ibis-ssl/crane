// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crane_geometry/ddps.hpp>
#include <crane_geometry/geometry_operations.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/ball_physics_model.hpp>
#include <crane_world_model_publisher/kick_event_detector.hpp>
#include <crane_world_model_publisher/pass_target_selector.hpp>
#include <crane_world_model_publisher/visualization_manager.hpp>
#include <crane_world_model_publisher/world_model_data_provider.hpp>
#include <crane_world_model_publisher/world_model_publisher.hpp>
#include <deque>
#include <filesystem>
#include <robocup_ssl_msgs/msg/robot_id.hpp>
#include <robocup_ssl_msgs/msg/ssl_detection_frame.hpp>

namespace crane
{
static auto parseStringToIntArray(const std::string & str) -> std::vector<uint8_t>
{
  std::vector<uint8_t> result;
  std::stringstream ss(str);
  int value;
  char comma;
  while (ss >> value) {
    result.push_back(static_cast<uint8_t>(value));
    // 次のカンマをスキップ（もしあれば）
    ss >> comma;
  }
  return result;
}

WorldModelPublisherComponent::WorldModelPublisherComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("world_model_publisher", options),
  data_provider_(std::make_unique<WorldModelDataProvider>(*this)),
  pub_world_model(this, "/world_model", 1, 50., 70.),
  diagnostic_updater_(this)
{
  using std::chrono_literals::operator""ms;

  // VisualizationManager初期化（統合された可視化システム）
  visualization_manager_ = std::make_unique<VisualizationManager>(*this);
  kick_event_detector_ = std::make_unique<KickEventDetector>();
  pass_target_selector_ = std::make_unique<PassTargetSelector>();

  // DataProviderのVisualization callbackをVisualizationManagerに接続
  data_provider_->setVisualizationCallbacks(
    [this](const robocup_ssl::SSL_GeometryData & geometry_data, bool half_court_mode) {
      visualization_manager_->drawFieldGeometry(geometry_data, half_court_mode);
    },
    [this](const robocup_ssl_msgs::msg::Referee & msg, double field_w, double field_h) {
      visualization_manager_->drawRefereeInfo(
        msg, field_w, field_h, data_provider_->getLatestPlaySituation().command.name);
    });

  declare_parameter("position_history_size", 200);
  get_parameter<int>("position_history_size", history_size);

  declare_parameter("robot_id_mask", std::string("1, 2, 3"));
  std::string robot_id_mask_str;
  get_parameter("robot_id_mask", robot_id_mask_str);
  data_provider_->setRobotIDsMask(parseStringToIntArray(robot_id_mask_str));

  // パス先スイッチ抑制のパラメータ
  declare_parameter("pass_target.min_hold_duration_sec", 0.5);
  declare_parameter("pass_target.min_improvement_margin", 0.2);
  double min_hold = 0.5, min_improve = 0.2;
  get_parameter("pass_target.min_hold_duration_sec", min_hold);
  get_parameter("pass_target.min_improvement_margin", min_improve);
  pass_target_selector_->setHysteresisParams(min_hold, min_improve);

  // ボール物理設定ファイルパス
  declare_parameter("ball_physics_config_path", std::string(""));
  std::string ball_physics_config_path;
  get_parameter("ball_physics_config_path", ball_physics_config_path);

  // ボール物理モデル初期化
  if (!ball_physics_config_path.empty()) {
    // ファイル名だけの場合はconfigディレクトリと結合
    std::string full_config_path = ball_physics_config_path;
    if (!std::filesystem::path(ball_physics_config_path).is_absolute()) {
      try {
        std::string package_share_dir =
          ament_index_cpp::get_package_share_directory("crane_world_model_publisher");
        full_config_path =
          std::filesystem::path(package_share_dir) / "config" / ball_physics_config_path;
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          this->get_logger(),
          "パッケージディレクトリの取得に失敗しました: %s 相対パスとして扱います", e.what());
      }
    }

    try {
      BallPhysicsModelFactory::createWithYAMLConfig(full_config_path);
      RCLCPP_INFO(
        this->get_logger(), "ボール物理設定を読み込みました: %s", full_config_path.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "ボール物理設定の読み込みに失敗しました (%s): %s デフォルト設定を使用します",
        full_config_path.c_str(), e.what());
      // エラー時はデフォルトファクトリーインスタンスを作成
      BallPhysicsModelFactory::getInstance();
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "ボール物理設定: デフォルト値を使用");
    BallPhysicsModelFactory::getInstance();
  }

  pub_process_time = create_publisher<std_msgs::msg::Float32>("~/process_time", 10);

  // 自動/world_modelサブスクライブはOFF
  wrapper_ = std::make_shared<WorldModelWrapper>(*this, false);

  // slack時間計算設定をWorldModelWrapperに設定（wrapper_初期化後）
  auto slack_config = SlackTimeConfig::fromNode(*this);
  wrapper_->setSlackConfig(slack_config);

  // デバッグ出力
  RCLCPP_INFO(get_logger(), "SlackTimeConfig loaded:");
  RCLCPP_INFO(
    get_logger(), "  robot_max_acceleration: %.2f m/s^2", slack_config.robot_max_acceleration);
  RCLCPP_INFO(get_logger(), "  robot_max_velocity: %.2f m/s", slack_config.robot_max_velocity);
  RCLCPP_INFO(get_logger(), "  time_horizon: %.2f s", slack_config.time_horizon);
  RCLCPP_INFO(get_logger(), "  time_step: %.2f s", slack_config.time_step);
  RCLCPP_INFO(get_logger(), "  slack_time_offset: %.2f s", slack_config.slack_time_offset);

  // 診断Updater設定
  diagnostic_updater_.setHardwareID("world_model_publisher");
  diagnostic_updater_.add(
    "vision/processing", this, &WorldModelPublisherComponent::updateDiagnostics);

  using std::chrono::operator""ms;
  timer = rclcpp::create_timer(this, get_clock(), 16ms, [this]() {
    bool available = data_provider_->available();
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000, "data_provider.available() = %s",
      available ? "true" : "false");

    if (available) {
      publishWorldModel();
      publishVisualization(wrapper_);
    } else {
      // より詳細な状態を表示
      bool has_vision = data_provider_->hasVisionUpdated();
      bool has_tracker = data_provider_->hasTrackedFrameUpdated();
      bool has_geometry = data_provider_->isGeometryInitialized();

      if (!has_vision && !has_tracker) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "Vision/Trackerデータを待機中 - world_modelは未発行");
      } else if (!has_geometry) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "フィールド情報を待機中 - world_modelは未発行 (Vision/Trackerデータは受信済み)");
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "データ準備中 - world_modelは未発行");
      }
    }

    // 診断情報を更新
    diagnostic_updater_.force_update();
  });
}

WorldModelPublisherComponent::~WorldModelPublisherComponent() = default;

auto WorldModelPublisherComponent::updateHistory(crane_msgs::msg::WorldModel & msg) -> void
{
  if (ball_info_history.size() >= static_cast<size_t>(history_size)) {
    ball_info_history.pop_front();
  }
  ball_info_history.emplace_back(msg.ball_info);

  for (const auto & robot : msg.robot_info_ours) {
    if (robot.available_vision || robot.available_feedback || robot.available_tracker) {
      friend_history[robot.id].push_back(robot);
    }
    if (friend_history[robot.id].size() > static_cast<size_t>(history_size)) {
      friend_history[robot.id].pop_front();
    }
  }

  for (const auto & robot : msg.robot_info_theirs) {
    if (robot.available_vision || robot.available_feedback || robot.available_tracker) {
      enemy_history[robot.id].push_back(robot);
    }
    if (enemy_history[robot.id].size() > static_cast<size_t>(history_size)) {
      enemy_history[robot.id].pop_front();
    }
  }
}

auto WorldModelPublisherComponent::publishWorldModel() -> void
{
  // 遅延監視: データ取得開始
  auto msg = data_provider_->getMsg();
  wrapper_->clearDelayCheckpoints();

  // VisionタイムスタンプをWorldModelWrapperに統合
  wrapper_->mergeDelayCheckpoints(msg.delay_checkpoints);

  // ROS 2でのVisionパケット受信時刻を追加
  wrapper_->addDelayCheckpoint("vision_packet_received", "ros2_received");
  wrapper_->addDelayCheckpoint("data_provider_getMsg", "vision_processed");

  updateHistory(msg);
  wrapper_->addDelayCheckpoint("history_updated", "");

  wrapper_->update(msg);
  wrapper_->addDelayCheckpoint("wrapper_updated", "");

  updateBallContact();
  wrapper_->addDelayCheckpoint("ball_contact_updated", "");

  postProcessWorldModel(wrapper_);
  wrapper_->addDelayCheckpoint("post_processed", "");

  pub_world_model.publish(wrapper_->getMsg());
  wrapper_->addDelayCheckpoint("world_model_published", "30Hz");
}

auto WorldModelPublisherComponent::publishVisualization(WorldModelWrapperPtr world_model) -> void
{
  // VisualizationManagerによる統合可視化処理
  visualization_manager_->drawTrackedObjects(world_model);

  // 軌跡履歴データをVisualizationManagerに渡す
  VisualizationManager::TrajectoryHistoryData trajectory_data;
  trajectory_data.friend_history = friend_history;
  trajectory_data.enemy_history = enemy_history;
  trajectory_data.ball_info_history = ball_info_history;
  trajectory_data.is_yellow = world_model->isYellow();

  visualization_manager_->drawTrajectoryHistory(trajectory_data);

  visualization_manager_->drawBallPlacement(world_model);

  crane::CraneVisualizerBuffer::publish();
}

auto WorldModelPublisherComponent::postProcessWorldModel(WorldModelWrapperPtr world_model) -> void
{
  kick_event_detector_->update(*world_model, visualization_manager_->kick_event_builder);
  crane_msgs::msg::GameAnalysis game_analysis_msg;
  if (auto kick = kick_event_detector_->getOnGoingKick(); kick.has_value()) {
    game_analysis_msg.ongoing_kick.push_back(*kick);
  }

  const auto & ball = world_model->ball();

  // ボールラインの長さを計算
  game_analysis_msg.ball_horizon = [&]() {
    Segment ball_line = ball.getTrajectorySegmentByTime(3.0);
    auto robots = world_model->theirs().getAvailableRobots();
    auto ball_line_lengths =
      robots |
      ranges::views::transform(
        [&](const auto & robot) { return getClosestPointAndDistance(ball_line, robot->pose.pos); })
      // 距離が0.5m以下のものを抽出
      | ranges::views::filter([](const ClosestPoint & pair) { return pair.distance < 0.5; })
      // ball.posとの距離を計算
      | ranges::views::transform([&](const ClosestPoint & pair) -> double {
          return (pair.closest_point - ball.pos).norm();
        });
    return ranges::empty(ball_line_lengths) ? 10.0 : ranges::min(ball_line_lengths);
  }();

  for (const auto & robot : wrapper_->ours().getAvailableRobots()) {
    RobotList single_robot{robot};
    auto [min_slack, max_slack] =
      world_model->getMinMaxSlackInterceptPointAndSlackTime(single_robot);
    crane_msgs::msg::Slack slack_msg;
    slack_msg.id = robot->id;
    if (min_slack) {
      slack_msg.min.slack_time = min_slack->slack_time;
      slack_msg.min.x = min_slack->intercept_point.x();
      slack_msg.min.y = min_slack->intercept_point.y();

      auto slack_builder = visualization_manager_->slack_builder;
      slack_builder->text()
        .position(robot->pose.pos.x(), robot->pose.pos.y() - 0.3)
        .text("min slack: " + std::to_string(min_slack->slack_time))
        .fill("white")
        .fontSize(100)
        .build();
      slack_builder->line()
        .start(robot->pose.pos)
        .end(min_slack->intercept_point)
        .stroke("red", 0.5)
        .strokeWidth(5)
        .build();
    }
    if (max_slack) {
      slack_msg.max.slack_time = max_slack->slack_time;
      slack_msg.max.x = max_slack->intercept_point.x();
      slack_msg.max.y = max_slack->intercept_point.y();

      if (max_slack->slack_time > 0.) {
        auto slack_builder = visualization_manager_->slack_builder;
        slack_builder->text()
          .position(robot->pose.pos.x(), robot->pose.pos.y() - 0.5)
          .text("max slack: " + std::to_string(max_slack->slack_time))
          .fill("white")
          .fontSize(100)
          .build();
        slack_builder->line()
          .start(robot->pose.pos)
          .end(max_slack->intercept_point)
          .stroke("red", 0.5)
          .strokeWidth(5)
          .build();
      }
    }
    game_analysis_msg.our_slack.push_back(slack_msg);
  }

  for (const auto & robot : wrapper_->theirs().getAvailableRobots()) {
    RobotList single_robot{robot};
    auto [min_slack, max_slack] =
      world_model->getMinMaxSlackInterceptPointAndSlackTime(single_robot);
    crane_msgs::msg::Slack slack_msg;
    slack_msg.id = robot->id;
    if (min_slack) {
      slack_msg.min.slack_time = min_slack->slack_time;
      slack_msg.min.x = min_slack->intercept_point.x();
      slack_msg.min.y = min_slack->intercept_point.y();
    }
    if (max_slack) {
      slack_msg.max.slack_time = max_slack->slack_time;
      slack_msg.max.x = max_slack->intercept_point.x();
      slack_msg.max.y = max_slack->intercept_point.y();
    }
    game_analysis_msg.their_slack.push_back(slack_msg);
  }

  // パススコア算出とパス先選定（専用クラスへ委譲）
  visualization_manager_->pass_score_builder->flush();
  pass_target_selector_->update(
    world_model, ball_info_history, visualization_manager_->pass_score_builder, game_analysis_msg);

  world_model->update(game_analysis_msg);
}

auto WorldModelPublisherComponent::updateBallContact() -> void
{
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();

  // ローカルセンサーの情報でボール情報を更新
  auto friend_robots = wrapper_->ours().getAvailableRobots();
  for (std::size_t i = 0; i < friend_robots.size(); i++) {
    auto robot = friend_robots[i];
    // ビジョンがボールを見失っているときに
    // ボールセンサが反応している間は、接触しているものとみなす。
    if (robot->getBallSensorAvailable(now) && not wrapper_->ball().detected) {
      // ビジョンはボール見失っているけどロボットが保持しているので、
      // ロボットの座標にボールがあることにする
      wrapper_->overwriteBallPos(robot->kicker_center());
    }
  }
}

auto WorldModelPublisherComponent::updateDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat) -> void
{
  // データが利用可能かチェック
  bool available = data_provider_->available();

  if (available) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Vision processing is running");

    // ボール検出状態
    if (wrapper_->ball().detected) {
      stat.add("ball_detected", "true");
    } else {
      stat.add("ball_detected", "false");
    }

    // 検出されたロボット数
    auto our_robots = wrapper_->ours().getAvailableRobots();
    auto their_robots = wrapper_->theirs().getAvailableRobots();
    stat.add("our_robots_count", static_cast<int>(our_robots.size()));
    stat.add("their_robots_count", static_cast<int>(their_robots.size()));
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No vision or tracker data available");
  }
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::WorldModelPublisherComponent)
