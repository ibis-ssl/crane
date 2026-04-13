// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_visualization_interfaces/crane_visualizer_wrapper.hpp>
#include <crane_world_model_publisher/visualization_manager.hpp>
#include <crane_world_model_publisher/world_model_data_provider.hpp>
#include <crane_world_model_publisher/world_model_publisher.hpp>
#include <deque>
#include <robocup_ssl_msgs/msg/ssl_detection_frame.hpp>
#include <sstream>

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
  diagnostic_helper_(
    this, "world_model_publisher", "vision/processing", this,
    &WorldModelPublisherComponent::updateDiagnostics),
  pub_world_model(this, "/world_model", 1, 50., 70.)
{
  using std::chrono_literals::operator""ms;

  // VisualizationManager初期化（統合された可視化システム）
  visualization_manager_ = std::make_unique<VisualizationManager>(*this);

  // DataProviderのVisualization callbackをVisualizationManagerに接続
  data_provider_->setVisualizationCallbacks(
    [this](const robocup_ssl::SSL_GeometryData & geometry_data, bool half_court_mode) {
      visualization_manager_->drawFieldGeometry(geometry_data, half_court_mode);
    },
    [this](const robocup_ssl_msgs::msg::Referee & msg, double field_w, double field_h) {
      visualization_manager_->drawRefereeInfo(
        msg, field_w, field_h, data_provider_->getLatestPlaySituation().command.name);
    });

  declare_parameter("robot_id_mask", std::string("1, 2, 3"));
  std::string robot_id_mask_str;
  get_parameter("robot_id_mask", robot_id_mask_str);
  data_provider_->setRobotIDsMask(parseStringToIntArray(robot_id_mask_str));

  // game_analysisを購読して、world_modelに引き継ぐ
  latest_game_analysis_msg_.pass_target_id = -1;
  latest_game_analysis_msg_.recommended_attacker_id = -1;
  latest_game_analysis_msg_.recommended_pass_receiver_id = -1;
  sub_game_analysis_ = create_subscription<crane_msgs::msg::GameAnalysis>(
    "/game_analysis", 10, [this](const crane_msgs::msg::GameAnalysis::SharedPtr msg) {
      std::scoped_lock lock(latest_game_analysis_msg_mutex_);
      latest_game_analysis_msg_ = *msg;
    });

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
    diagnostic_helper_.forceUpdate();
  });
}

WorldModelPublisherComponent::~WorldModelPublisherComponent() = default;

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
  // チーム色情報を更新
  visualization_manager_->updateTeamInfo(world_model->isYellow(), world_model->onPositiveHalf());

  // VisualizationManagerによる統合可視化処理
  visualization_manager_->drawTrackedObjects(world_model);

  visualization_manager_->drawBallPlacement(world_model);

  crane::CraneVisualizerBuffer::publish();
}

auto WorldModelPublisherComponent::postProcessWorldModel(WorldModelWrapperPtr world_model) -> void
{
  crane_msgs::msg::GameAnalysis game_analysis_msg;
  {
    std::scoped_lock lock(latest_game_analysis_msg_mutex_);
    game_analysis_msg = latest_game_analysis_msg_;
  }
  world_model->update(game_analysis_msg);
}

auto WorldModelPublisherComponent::updateBallContact() -> void
{
  auto now = this->now();
  auto friend_robots = wrapper_->ours().robotsWhere().available().get();
  for (std::size_t i = 0; i < friend_robots.size(); i++) {
    auto robot = friend_robots[i];
    if (robot->getBallSensorAvailable(now) && not wrapper_->ball().detected) {
      const auto kicker_pos = robot->kicker_center();
      data_provider_->setBallSensorHint(
        static_cast<uint32_t>(robot->id), Eigen::Vector2d(kicker_pos.x(), kicker_pos.y()), now);
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
    auto our_robots = wrapper_->ours().robotsWhere().available().get();
    auto their_robots = wrapper_->theirs().robotsWhere().available().get();
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
