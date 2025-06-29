// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/tracker_manager_factory.hpp"

namespace crane
{

TrackerManagerFactory::TrackerManagerFactory(rclcpp::Node & node) : node_(node)
{
  // デフォルト設定のロード
  loadConfigFromParameters();
}

auto TrackerManagerFactory::createBallTrackerManager() -> std::unique_ptr<BallTrackerManager>
{
  return std::make_unique<BallTrackerManager>(node_.get_clock());
}

auto TrackerManagerFactory::createRobotTrackerManager() -> std::unique_ptr<RobotTrackerManager>
{
  return std::make_unique<RobotTrackerManager>(node_.get_clock());
}

auto TrackerManagerFactory::getSharedBallTrackerManager() -> std::shared_ptr<BallTrackerManager>
{
  if (!shared_ball_tracker_) {
    shared_ball_tracker_ = std::shared_ptr<BallTrackerManager>(createBallTrackerManager().release());
  }
  return shared_ball_tracker_;
}

auto TrackerManagerFactory::getSharedRobotTrackerManager() -> std::shared_ptr<RobotTrackerManager>
{
  if (!shared_robot_tracker_) {
    shared_robot_tracker_ = std::shared_ptr<RobotTrackerManager>(createRobotTrackerManager().release());
  }
  return shared_robot_tracker_;
}

auto TrackerManagerFactory::setTrackerConfig(const TrackerManagerConfig & config) -> void
{
  config_ = config;
  
  // 既存の共有インスタンスがある場合は再初期化
  if (shared_ball_tracker_ || shared_robot_tracker_) {
    initializeSharedInstances();
  }
}

auto TrackerManagerFactory::getTrackerConfig() const -> const TrackerManagerConfig &
{
  return config_;
}

auto TrackerManagerFactory::loadConfigFromParameters() -> void
{
  // ROS 2パラメータからの設定読み込み
  node_.declare_parameter("tracker.ball_outlier_threshold", config_.ball_outlier_threshold);
  node_.declare_parameter("tracker.ball_min_confidence", config_.ball_min_confidence);
  node_.declare_parameter("tracker.ball_max_age_seconds", config_.ball_max_age_seconds);
  
  node_.declare_parameter("tracker.robot_outlier_threshold", config_.robot_outlier_threshold);
  node_.declare_parameter("tracker.robot_min_confidence", config_.robot_min_confidence);
  node_.declare_parameter("tracker.robot_max_age_seconds", config_.robot_max_age_seconds);
  
  node_.declare_parameter("tracker.enable_physics_validation", config_.enable_physics_validation);
  node_.declare_parameter("tracker.enable_multi_ball_tracking", config_.enable_multi_ball_tracking);
  
  // パラメータ値の取得
  config_.ball_outlier_threshold = node_.get_parameter("tracker.ball_outlier_threshold").as_double();
  config_.ball_min_confidence = node_.get_parameter("tracker.ball_min_confidence").as_double();
  config_.ball_max_age_seconds = node_.get_parameter("tracker.ball_max_age_seconds").as_double();
  
  config_.robot_outlier_threshold = node_.get_parameter("tracker.robot_outlier_threshold").as_double();
  config_.robot_min_confidence = node_.get_parameter("tracker.robot_min_confidence").as_double();
  config_.robot_max_age_seconds = node_.get_parameter("tracker.robot_max_age_seconds").as_double();
  
  config_.enable_physics_validation = node_.get_parameter("tracker.enable_physics_validation").as_bool();
  config_.enable_multi_ball_tracking = node_.get_parameter("tracker.enable_multi_ball_tracking").as_bool();
  
  RCLCPP_INFO(node_.get_logger(), 
    "TrackerManagerFactory: Loaded config - Ball threshold: %.2f, Robot threshold: %.2f",
    config_.ball_outlier_threshold, config_.robot_outlier_threshold);
}

auto TrackerManagerFactory::initializeSharedInstances() -> void
{
  // 共有インスタンスの再作成
  shared_ball_tracker_.reset();
  shared_robot_tracker_.reset();
  
  // 必要に応じて遅延初期化される
}

// TrackerManagerContainer implementation
TrackerManagerContainer::TrackerManagerContainer(rclcpp::Node & node) : node_(node)
{
  factory_ = std::make_unique<TrackerManagerFactory>(node_);
}

auto TrackerManagerContainer::getBallTrackerManager() -> std::shared_ptr<BallTrackerManager>
{
  if (!initialized_) {
    initialize();
  }
  return ball_tracker_manager_;
}

auto TrackerManagerContainer::getRobotTrackerManager() -> std::shared_ptr<RobotTrackerManager>
{
  if (!initialized_) {
    initialize();
  }
  return robot_tracker_manager_;
}

auto TrackerManagerContainer::setTrackerConfig(const TrackerManagerConfig & config) -> void
{
  factory_->setTrackerConfig(config);
  
  // 既に初期化済みの場合は再初期化
  if (initialized_) {
    initialize();
  }
}

auto TrackerManagerContainer::initialize() -> void
{
  ball_tracker_manager_ = factory_->getSharedBallTrackerManager();
  robot_tracker_manager_ = factory_->getSharedRobotTrackerManager();
  
  initialized_ = true;
  
  RCLCPP_INFO(node_.get_logger(), "TrackerManagerContainer: Initialized successfully");
}

auto TrackerManagerContainer::connectToVisionProcessor() -> void
{
  // VisionDataProcessorとの連携設定
  // 実際の実装では、VisionDataProcessorのコールバック設定をここで行う
  RCLCPP_DEBUG(node_.get_logger(), "TrackerManagerContainer: Connected to VisionProcessor");
}

auto TrackerManagerContainer::connectToDataSourceManager() -> void
{
  // DataSourceManagerとの連携設定
  RCLCPP_DEBUG(node_.get_logger(), "TrackerManagerContainer: Connected to DataSourceManager");
}

auto TrackerManagerContainer::predict(double dt) -> void
{
  if (ball_tracker_manager_) {
    ball_tracker_manager_->predict(dt);
  }
  
  if (robot_tracker_manager_) {
    robot_tracker_manager_->predict(dt);
  }
}

auto TrackerManagerContainer::removeOldTrackers() -> void
{
  const auto & config = factory_->getTrackerConfig();
  
  if (ball_tracker_manager_) {
    ball_tracker_manager_->removeOldTrackers(config.ball_max_age_seconds);
  }
  
  if (robot_tracker_manager_) {
    robot_tracker_manager_->removeOldTrackers(config.robot_max_age_seconds);
  }
}

// TrackerServiceImplementation
TrackerServiceImplementation::TrackerServiceImplementation(
  std::shared_ptr<TrackerManagerContainer> container)
: container_(container)
{
}

auto TrackerServiceImplementation::processBallDetection(
  const Vector3 & position, const rclcpp::Time & timestamp) 
  -> crane_msgs::msg::BallInfo
{
  auto ball_manager = container_->getBallTrackerManager();
  if (ball_manager) {
    return ball_manager->processVisionDetection(position, timestamp);
  }
  
  // フォールバック: 空のBallInfo
  crane_msgs::msg::BallInfo empty_ball_info;
  empty_ball_info.detected = false;
  return empty_ball_info;
}

auto TrackerServiceImplementation::getBestBallTracker() -> std::shared_ptr<BallTracker>
{
  auto ball_manager = container_->getBallTrackerManager();
  return ball_manager ? ball_manager->getBestTracker() : nullptr;
}

auto TrackerServiceImplementation::processRobotDetection(
  uint8_t robot_id, RobotTrackerType type, const Vector3 & pose, const rclcpp::Time & timestamp)
  -> void
{
  auto robot_manager = container_->getRobotTrackerManager();
  if (robot_manager) {
    robot_manager->processVisionDetection(robot_id, type, pose, timestamp);
  }
}

auto TrackerServiceImplementation::getRobotTracker(uint8_t robot_id, RobotTrackerType type) 
  -> std::shared_ptr<RobotTracker>
{
  auto robot_manager = container_->getRobotTrackerManager();
  return robot_manager ? robot_manager->getRobotTracker(robot_id, type) : nullptr;
}

auto TrackerServiceImplementation::getAllRobotInfo() -> std::vector<crane_msgs::msg::RobotInfo>
{
  auto robot_manager = container_->getRobotTrackerManager();
  return robot_manager ? robot_manager->getAllRobotInfo() : std::vector<crane_msgs::msg::RobotInfo>{};
}

auto TrackerServiceImplementation::updateRobotFeedback(
  uint8_t robot_id, const crane_msgs::msg::RobotFeedback & feedback) -> void
{
  auto robot_manager = container_->getRobotTrackerManager();
  if (robot_manager) {
    robot_manager->updateFriendlyRobotFeedback(robot_id, feedback);
  }
}

auto TrackerServiceImplementation::updateRobotCommand(
  uint8_t robot_id, const Vector2 & cmd_vel, double cmd_omega) -> void
{
  auto robot_manager = container_->getRobotTrackerManager();
  if (robot_manager) {
    robot_manager->updateFriendlyRobotCommand(robot_id, cmd_vel, cmd_omega);
  }
}

auto TrackerServiceImplementation::predict(double dt) -> void
{
  container_->predict(dt);
}

auto TrackerServiceImplementation::removeOldTrackers() -> void
{
  container_->removeOldTrackers();
}

}  // namespace crane