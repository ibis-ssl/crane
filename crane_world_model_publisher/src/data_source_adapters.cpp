// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/data_source_adapters.hpp"

#include <algorithm>
#include <mutex>

namespace crane
{
// VisionDataSourceAdapter implementation
VisionDataSourceAdapter::VisionDataSourceAdapter(
  std::shared_ptr<VisionDataProcessor> vision_processor)
: vision_processor_(vision_processor), is_active_(true), metadata_initialized_(false)
{
  if (!vision_processor_) {
    throw std::invalid_argument("VisionDataProcessor cannot be null");
  }
}

auto VisionDataSourceAdapter::getMetadata() const -> const DataSourceMetadata &
{
  if (!metadata_initialized_) {
    initializeMetadata();
  }
  return cached_metadata_;
}

auto VisionDataSourceAdapter::initializeMetadata() const -> void
{
  cached_metadata_.name = "SSL-Vision";
  cached_metadata_.description = "SSL-Vision camera system with EKF ball/robot tracking";
  cached_metadata_.priority = DataSourcePriority::PRIMARY;
  cached_metadata_.capabilities = 0;

  addCapability(cached_metadata_.capabilities, DataSourceCapability::BALL_TRACKING);
  addCapability(cached_metadata_.capabilities, DataSourceCapability::ROBOT_TRACKING);
  addCapability(cached_metadata_.capabilities, DataSourceCapability::FIELD_GEOMETRY);
  addCapability(cached_metadata_.capabilities, DataSourceCapability::REAL_TIME_TRACKING);
  addCapability(cached_metadata_.capabilities, DataSourceCapability::HIGH_ACCURACY);

  cached_metadata_.update_frequency_hz = 60.0;  // SSL-Vision typical frequency
  cached_metadata_.accuracy_estimate_meters = 0.01;  // 1cm accuracy
  cached_metadata_.last_update_time = rclcpp::Clock(RCL_ROS_TIME).now();
  cached_metadata_.is_active = is_active_;

  metadata_initialized_ = true;
}

auto VisionDataSourceAdapter::isAvailable() const -> bool
{
  return vision_processor_ && vision_processor_->hasVisionUpdated();
}

auto VisionDataSourceAdapter::hasRecentUpdate(double max_age_seconds) const -> bool
{
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();
  auto age = (now - cached_metadata_.last_update_time).seconds();
  return age <= max_age_seconds;
}

auto VisionDataSourceAdapter::activate() -> void { is_active_ = true; }

auto VisionDataSourceAdapter::deactivate() -> void { is_active_ = false; }

auto VisionDataSourceAdapter::reset() -> void
{
  // Vision processor doesn't have an explicit reset method
  // Could potentially implement restart logic here
}

auto VisionDataSourceAdapter::getQualityScore() const -> double
{
  if (!isAvailable()) {
    return 0.0;
  }

  double score = 1.0;  // Start with maximum quality

  // Reduce score based on age of last update
  if (!hasRecentUpdate(1.0)) {
    score *= 0.5;  // Significant reduction for stale data
  } else if (!hasRecentUpdate(0.1)) {
    score *= 0.8;  // Moderate reduction for slightly old data
  }

  return score;
}

auto VisionDataSourceAdapter::getLatencyMilliseconds() const -> double
{
  // Estimate latency based on vision processing pipeline
  // This could be made more accurate by tracking actual timestamps
  return 15.0;  // Typical SSL-Vision processing latency
}

auto VisionDataSourceAdapter::getBallInfo() const -> crane_msgs::msg::BallInfo
{
  return vision_processor_->getBallInfo();
}

auto VisionDataSourceAdapter::hasBallUpdate() const -> bool
{
  return vision_processor_->hasVisionUpdated();
}

auto VisionDataSourceAdapter::getBallConfidence() const -> double
{
  // Vision processor doesn't expose confidence directly
  // Estimate based on availability and recency
  return isAvailable() ? getQualityScore() : 0.0;
}

auto VisionDataSourceAdapter::getBallTrackingState() const -> std::string
{
  if (isAvailable()) {
    return "TRACKING";
  } else {
    return "NO_DATA";
  }
}

auto VisionDataSourceAdapter::getRobotInfo(int team_index) const
  -> std::vector<crane_msgs::msg::RobotInfo>
{
  return vision_processor_->getRobotInfo(team_index);
}

auto VisionDataSourceAdapter::hasRobotUpdate() const -> bool
{
  return vision_processor_->hasVisionUpdated();
}

auto VisionDataSourceAdapter::getRobotConfidence(int team_index, uint8_t robot_id) const
  -> double
{
  // Get robot info and check if the specific robot is detected
  auto robots = vision_processor_->getRobotInfo(team_index);
  for (const auto & robot : robots) {
    if (robot.id == robot_id) {
      return robot.vision_detected ? getQualityScore() : 0.0;
    }
  }
  return 0.0;
}

auto VisionDataSourceAdapter::getRobotTrackingState(int team_index, uint8_t robot_id) const
  -> std::string
{
  auto robots = vision_processor_->getRobotInfo(team_index);
  for (const auto & robot : robots) {
    if (robot.id == robot_id) {
      return robot.vision_detected ? "TRACKING" : "NOT_DETECTED";
    }
  }
  return "NOT_FOUND";
}

auto VisionDataSourceAdapter::getFieldWidth() const -> double
{
  return vision_processor_->getFieldWidth();
}

auto VisionDataSourceAdapter::getFieldHeight() const -> double
{
  return vision_processor_->getFieldHeight();
}

auto VisionDataSourceAdapter::getGoalWidth() const -> double
{
  return vision_processor_->getGoalWidth();
}

auto VisionDataSourceAdapter::getGoalHeight() const -> double
{
  return vision_processor_->getGoalHeight();
}

auto VisionDataSourceAdapter::getPenaltyAreaWidth() const -> double
{
  return vision_processor_->getPenaltyAreaWidth();
}

auto VisionDataSourceAdapter::getPenaltyAreaHeight() const -> double
{
  return vision_processor_->getPenaltyAreaHeight();
}

auto VisionDataSourceAdapter::hasGeometryUpdate() const -> bool
{
  return vision_processor_->hasVisionUpdated();
}

auto VisionDataSourceAdapter::getSupportedDataTypes() const -> uint32_t
{
  return getMetadata().capabilities;
}

auto VisionDataSourceAdapter::canReplaceDataSource(const DataSourceBase & other) const -> bool
{
  const auto & other_meta = other.getMetadata();
  return hasCapability(other_meta.capabilities, DataSourceCapability::BALL_TRACKING) ||
         hasCapability(other_meta.capabilities, DataSourceCapability::ROBOT_TRACKING) ||
         hasCapability(other_meta.capabilities, DataSourceCapability::FIELD_GEOMETRY);
}

// TrackerDataSourceAdapter implementation
TrackerDataSourceAdapter::TrackerDataSourceAdapter(
  std::shared_ptr<TrackerDataProcessor> tracker_processor)
: tracker_processor_(tracker_processor), is_active_(true), metadata_initialized_(false)
{
  if (!tracker_processor_) {
    throw std::invalid_argument("TrackerDataProcessor cannot be null");
  }
}

auto TrackerDataSourceAdapter::getMetadata() const -> const DataSourceMetadata &
{
  if (!metadata_initialized_) {
    initializeMetadata();
  }
  return cached_metadata_;
}

auto TrackerDataSourceAdapter::initializeMetadata() const -> void
{
  cached_metadata_.name = "SSL-Tracker";
  cached_metadata_.description = "External SSL-Tracker for fallback tracking";
  cached_metadata_.priority = DataSourcePriority::SECONDARY;
  cached_metadata_.capabilities = 0;

  addCapability(cached_metadata_.capabilities, DataSourceCapability::BALL_TRACKING);
  addCapability(cached_metadata_.capabilities, DataSourceCapability::ROBOT_TRACKING);
  addCapability(cached_metadata_.capabilities, DataSourceCapability::REAL_TIME_TRACKING);

  cached_metadata_.update_frequency_hz = 60.0;
  cached_metadata_.accuracy_estimate_meters = 0.02;  // Slightly lower accuracy than vision
  cached_metadata_.last_update_time = rclcpp::Clock(RCL_ROS_TIME).now();
  cached_metadata_.is_active = is_active_;

  metadata_initialized_ = true;
}

auto TrackerDataSourceAdapter::isAvailable() const -> bool
{
  return tracker_processor_ && tracker_processor_->hasTrackerUpdated();
}

auto TrackerDataSourceAdapter::hasRecentUpdate(double max_age_seconds) const -> bool
{
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();
  auto age = (now - cached_metadata_.last_update_time).seconds();
  return age <= max_age_seconds;
}

auto TrackerDataSourceAdapter::activate() -> void { is_active_ = true; }

auto TrackerDataSourceAdapter::deactivate() -> void { is_active_ = false; }

auto TrackerDataSourceAdapter::reset() -> void
{
  // Tracker processor doesn't have an explicit reset method
}

auto TrackerDataSourceAdapter::getQualityScore() const -> double
{
  if (!isAvailable()) {
    return 0.0;
  }

  double score = 0.8;  // Start lower than vision due to secondary priority

  if (!hasRecentUpdate(1.0)) {
    score *= 0.5;
  } else if (!hasRecentUpdate(0.1)) {
    score *= 0.8;
  }

  return score;
}

auto TrackerDataSourceAdapter::getLatencyMilliseconds() const -> double
{
  return 20.0;  // Slightly higher latency than vision
}

auto TrackerDataSourceAdapter::getBallInfo() const -> crane_msgs::msg::BallInfo
{
  return tracker_processor_->getBallInfo();
}

auto TrackerDataSourceAdapter::hasBallUpdate() const -> bool
{
  return tracker_processor_->hasTrackerUpdated();
}

auto TrackerDataSourceAdapter::getBallConfidence() const -> double
{
  return isAvailable() ? getQualityScore() : 0.0;
}

auto TrackerDataSourceAdapter::getBallTrackingState() const -> std::string
{
  return isAvailable() ? "TRACKING" : "NO_DATA";
}

auto TrackerDataSourceAdapter::getRobotInfo(int team_index) const
  -> std::vector<crane_msgs::msg::RobotInfo>
{
  return tracker_processor_->getRobotInfo(team_index);
}

auto TrackerDataSourceAdapter::hasRobotUpdate() const -> bool
{
  return tracker_processor_->hasTrackerUpdated();
}

auto TrackerDataSourceAdapter::getRobotConfidence(int team_index, uint8_t robot_id) const
  -> double
{
  auto robots = tracker_processor_->getRobotInfo(team_index);
  for (const auto & robot : robots) {
    if (robot.id == robot_id) {
      return robot.vision_detected ? getQualityScore() : 0.0;
    }
  }
  return 0.0;
}

auto TrackerDataSourceAdapter::getRobotTrackingState(int team_index, uint8_t robot_id) const
  -> std::string
{
  auto robots = tracker_processor_->getRobotInfo(team_index);
  for (const auto & robot : robots) {
    if (robot.id == robot_id) {
      return robot.vision_detected ? "TRACKING" : "NOT_DETECTED";
    }
  }
  return "NOT_FOUND";
}

// Tracker doesn't provide geometry data
auto TrackerDataSourceAdapter::getFieldWidth() const -> double { return 0.0; }
auto TrackerDataSourceAdapter::getFieldHeight() const -> double { return 0.0; }
auto TrackerDataSourceAdapter::getGoalWidth() const -> double { return 0.0; }
auto TrackerDataSourceAdapter::getGoalHeight() const -> double { return 0.0; }
auto TrackerDataSourceAdapter::getPenaltyAreaWidth() const -> double { return 0.0; }
auto TrackerDataSourceAdapter::getPenaltyAreaHeight() const -> double { return 0.0; }
auto TrackerDataSourceAdapter::hasGeometryUpdate() const -> bool { return false; }

auto TrackerDataSourceAdapter::getSupportedDataTypes() const -> uint32_t
{
  return getMetadata().capabilities;
}

auto TrackerDataSourceAdapter::canReplaceDataSource(const DataSourceBase & other) const -> bool
{
  const auto & other_meta = other.getMetadata();
  return hasCapability(other_meta.capabilities, DataSourceCapability::BALL_TRACKING) ||
         hasCapability(other_meta.capabilities, DataSourceCapability::ROBOT_TRACKING);
}

// DataSourceAdapterFactory implementation
DataSourceAdapterFactory::DataSourceAdapterFactory(rclcpp::Node & node) : node_(node) {}

auto DataSourceAdapterFactory::createBallDataSource(const std::string & config) const
  -> std::unique_ptr<BallDataSource>
{
  if (config == "vision") {
    auto vision_adapter = createVisionDataSourceAdapter();
    return std::unique_ptr<BallDataSource>(vision_adapter.release());
  } else if (config == "tracker") {
    auto tracker_adapter = createTrackerDataSourceAdapter();
    return std::unique_ptr<BallDataSource>(tracker_adapter.release());
  }
  return nullptr;
}

auto DataSourceAdapterFactory::createRobotDataSource(const std::string & config) const
  -> std::unique_ptr<RobotDataSource>
{
  if (config == "vision") {
    auto vision_adapter = createVisionDataSourceAdapter();
    return std::unique_ptr<RobotDataSource>(vision_adapter.release());
  } else if (config == "tracker") {
    auto tracker_adapter = createTrackerDataSourceAdapter();
    return std::unique_ptr<RobotDataSource>(tracker_adapter.release());
  }
  return nullptr;
}

auto DataSourceAdapterFactory::createCompositeDataSource(const std::string & config) const
  -> std::unique_ptr<CompositeDataSource>
{
  if (config == "vision") {
    return createVisionDataSourceAdapter();
  } else if (config == "tracker") {
    return createTrackerDataSourceAdapter();
  }
  return nullptr;
}

auto DataSourceAdapterFactory::listAvailableDataSources() const -> std::vector<std::string>
{
  return {"vision", "tracker"};
}

auto DataSourceAdapterFactory::validateConfiguration(const std::string & config) const -> bool
{
  auto available = listAvailableDataSources();
  return std::find(available.begin(), available.end(), config) != available.end();
}

auto DataSourceAdapterFactory::createVisionDataSourceAdapter() const
  -> std::unique_ptr<VisionDataSourceAdapter>
{
  auto vision_processor = std::make_shared<VisionDataProcessor>(node_);
  return std::make_unique<VisionDataSourceAdapter>(vision_processor);
}

auto DataSourceAdapterFactory::createTrackerDataSourceAdapter() const
  -> std::unique_ptr<TrackerDataSourceAdapter>
{
  auto tracker_processor = std::make_shared<TrackerDataProcessor>(node_);
  return std::make_unique<TrackerDataSourceAdapter>(tracker_processor);
}

// DataSourceRegistry implementation
DataSourceRegistry::DataSourceRegistry() {}

auto DataSourceRegistry::registerBallDataSource(
  const std::string & name, std::unique_ptr<BallDataSource> source) -> void
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  ball_sources_[name] = std::move(source);
}

auto DataSourceRegistry::registerRobotDataSource(
  const std::string & name, std::unique_ptr<RobotDataSource> source) -> void
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  robot_sources_[name] = std::move(source);
}

auto DataSourceRegistry::registerCompositeDataSource(
  const std::string & name, std::unique_ptr<CompositeDataSource> source) -> void
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  composite_sources_[name] = std::move(source);
}

auto DataSourceRegistry::getBallDataSource(const std::string & name) const
  -> std::shared_ptr<BallDataSource>
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  auto it = ball_sources_.find(name);
  return (it != ball_sources_.end()) ? it->second : nullptr;
}

auto DataSourceRegistry::getRobotDataSource(const std::string & name) const
  -> std::shared_ptr<RobotDataSource>
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  auto it = robot_sources_.find(name);
  return (it != robot_sources_.end()) ? it->second : nullptr;
}

auto DataSourceRegistry::getCompositeDataSource(const std::string & name) const
  -> std::shared_ptr<CompositeDataSource>
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  auto it = composite_sources_.find(name);
  return (it != composite_sources_.end()) ? it->second : nullptr;
}

auto DataSourceRegistry::listBallDataSources() const -> std::vector<std::string>
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  std::vector<std::string> names;
  for (const auto & pair : ball_sources_) {
    names.push_back(pair.first);
  }
  return names;
}

auto DataSourceRegistry::listRobotDataSources() const -> std::vector<std::string>
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  std::vector<std::string> names;
  for (const auto & pair : robot_sources_) {
    names.push_back(pair.first);
  }
  return names;
}

auto DataSourceRegistry::listCompositeDataSources() const -> std::vector<std::string>
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  std::vector<std::string> names;
  for (const auto & pair : composite_sources_) {
    names.push_back(pair.first);
  }
  return names;
}

auto DataSourceRegistry::selectBestBallDataSource() const -> std::shared_ptr<BallDataSource>
{
  std::lock_guard<std::mutex> lock(registry_mutex_);

  std::shared_ptr<BallDataSource> best_source = nullptr;
  DataSourceComparator comparator;

  for (const auto & pair : ball_sources_) {
    if (pair.second->isAvailable()) {
      if (!best_source || comparator(*pair.second, *best_source)) {
        best_source = pair.second;
      }
    }
  }

  return best_source;
}

auto DataSourceRegistry::selectBestRobotDataSource() const -> std::shared_ptr<RobotDataSource>
{
  std::lock_guard<std::mutex> lock(registry_mutex_);

  std::shared_ptr<RobotDataSource> best_source = nullptr;
  DataSourceComparator comparator;

  for (const auto & pair : robot_sources_) {
    if (pair.second->isAvailable()) {
      if (!best_source || comparator(*pair.second, *best_source)) {
        best_source = pair.second;
      }
    }
  }

  return best_source;
}

auto DataSourceRegistry::performHealthCheck() -> void
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  // Implementation would check all sources for health
  // For now, this is a placeholder
}

auto DataSourceRegistry::getHealthStatus() const -> std::string
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  return "OK";  // Placeholder implementation
}
}  // namespace crane