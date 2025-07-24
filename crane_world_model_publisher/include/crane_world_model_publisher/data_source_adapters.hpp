// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__DATA_SOURCE_ADAPTERS_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__DATA_SOURCE_ADAPTERS_HPP_

#include <crane_world_model_publisher/data_source_interface.hpp>
#include <crane_world_model_publisher/tracker_data_processor.hpp>
#include <crane_world_model_publisher/vision_data_processor.hpp>
#include <memory>

namespace crane
{
class VisionDataSourceAdapter : public CompositeDataSource
{
public:
  explicit VisionDataSourceAdapter(std::shared_ptr<VisionDataProcessor> vision_processor);
  ~VisionDataSourceAdapter() override = default;

  // DataSourceBase interface
  [[nodiscard]] auto getMetadata() const -> const DataSourceMetadata & override;
  [[nodiscard]] auto isAvailable() const -> bool override;
  [[nodiscard]] auto hasRecentUpdate(double max_age_seconds = 1.0) const -> bool override;
  auto activate() -> void override;
  auto deactivate() -> void override;
  auto reset() -> void override;
  [[nodiscard]] auto getQualityScore() const -> double override;
  [[nodiscard]] auto getLatencyMilliseconds() const -> double override;

  // BallDataSource interface
  [[nodiscard]] auto getBallInfo() const -> crane_msgs::msg::BallInfo override;
  [[nodiscard]] auto hasBallUpdate() const -> bool override;
  [[nodiscard]] auto getBallConfidence() const -> double override;
  [[nodiscard]] auto getBallTrackingState() const -> std::string override;

  // RobotDataSource interface
  [[nodiscard]] auto getRobotInfo(int team_index) const
    -> std::vector<crane_msgs::msg::RobotInfo> override;
  [[nodiscard]] auto hasRobotUpdate() const -> bool override;
  [[nodiscard]] auto getRobotConfidence(int team_index, uint8_t robot_id) const -> double override;
  [[nodiscard]] auto getRobotTrackingState(int team_index, uint8_t robot_id) const
    -> std::string override;

  // FieldGeometryDataSource interface
  [[nodiscard]] auto getFieldWidth() const -> double override;
  [[nodiscard]] auto getFieldHeight() const -> double override;
  [[nodiscard]] auto getGoalWidth() const -> double override;
  [[nodiscard]] auto getGoalHeight() const -> double override;
  [[nodiscard]] auto getPenaltyAreaWidth() const -> double override;
  [[nodiscard]] auto getPenaltyAreaHeight() const -> double override;
  [[nodiscard]] auto hasGeometryUpdate() const -> bool override;

  // CompositeDataSource interface
  [[nodiscard]] auto getSupportedDataTypes() const -> uint32_t override;
  [[nodiscard]] auto canReplaceDataSource(const DataSourceBase & other) const -> bool override;

  // Adapter-specific methods
  [[nodiscard]] auto getVisionProcessor() const -> std::shared_ptr<VisionDataProcessor>
  {
    return vision_processor_;
  }

private:
  std::shared_ptr<VisionDataProcessor> vision_processor_;
  bool is_active_;
  mutable DataSourceMetadata cached_metadata_;
  mutable bool metadata_initialized_;

  auto initializeMetadata() const -> void;
};

class TrackerDataSourceAdapter : public CompositeDataSource
{
public:
  explicit TrackerDataSourceAdapter(std::shared_ptr<TrackerDataProcessor> tracker_processor);
  ~TrackerDataSourceAdapter() override = default;

  // DataSourceBase interface
  [[nodiscard]] auto getMetadata() const -> const DataSourceMetadata & override;
  [[nodiscard]] auto isAvailable() const -> bool override;
  [[nodiscard]] auto hasRecentUpdate(double max_age_seconds = 1.0) const -> bool override;
  auto activate() -> void override;
  auto deactivate() -> void override;
  auto reset() -> void override;
  [[nodiscard]] auto getQualityScore() const -> double override;
  [[nodiscard]] auto getLatencyMilliseconds() const -> double override;

  // BallDataSource interface
  [[nodiscard]] auto getBallInfo() const -> crane_msgs::msg::BallInfo override;
  [[nodiscard]] auto hasBallUpdate() const -> bool override;
  [[nodiscard]] auto getBallConfidence() const -> double override;
  [[nodiscard]] auto getBallTrackingState() const -> std::string override;

  // RobotDataSource interface
  [[nodiscard]] auto getRobotInfo(int team_index) const
    -> std::vector<crane_msgs::msg::RobotInfo> override;
  [[nodiscard]] auto hasRobotUpdate() const -> bool override;
  [[nodiscard]] auto getRobotConfidence(int team_index, uint8_t robot_id) const -> double override;
  [[nodiscard]] auto getRobotTrackingState(int team_index, uint8_t robot_id) const
    -> std::string override;

  // FieldGeometryDataSource interface (tracker typically doesn't provide geometry)
  [[nodiscard]] auto getFieldWidth() const -> double override;
  [[nodiscard]] auto getFieldHeight() const -> double override;
  [[nodiscard]] auto getGoalWidth() const -> double override;
  [[nodiscard]] auto getGoalHeight() const -> double override;
  [[nodiscard]] auto getPenaltyAreaWidth() const -> double override;
  [[nodiscard]] auto getPenaltyAreaHeight() const -> double override;
  [[nodiscard]] auto hasGeometryUpdate() const -> bool override;

  // CompositeDataSource interface
  [[nodiscard]] auto getSupportedDataTypes() const -> uint32_t override;
  [[nodiscard]] auto canReplaceDataSource(const DataSourceBase & other) const -> bool override;

  // Adapter-specific methods
  [[nodiscard]] auto getTrackerProcessor() const -> std::shared_ptr<TrackerDataProcessor>
  {
    return tracker_processor_;
  }

private:
  std::shared_ptr<TrackerDataProcessor> tracker_processor_;
  bool is_active_;
  mutable DataSourceMetadata cached_metadata_;
  mutable bool metadata_initialized_;

  auto initializeMetadata() const -> void;
};

// Factory for creating adapter instances
class DataSourceAdapterFactory : public DataSourceFactory
{
public:
  explicit DataSourceAdapterFactory(rclcpp::Node & node);
  ~DataSourceAdapterFactory() override = default;

  // Factory methods
  [[nodiscard]] auto createBallDataSource(const std::string & config) const
    -> std::unique_ptr<BallDataSource> override;
  [[nodiscard]] auto createRobotDataSource(const std::string & config) const
    -> std::unique_ptr<RobotDataSource> override;
  [[nodiscard]] auto createCompositeDataSource(const std::string & config) const
    -> std::unique_ptr<CompositeDataSource> override;

  // Discovery methods
  [[nodiscard]] auto listAvailableDataSources() const -> std::vector<std::string> override;
  [[nodiscard]] auto validateConfiguration(const std::string & config) const -> bool override;

  // Convenience methods for creating specific adapters
  [[nodiscard]] auto createVisionDataSourceAdapter() const
    -> std::unique_ptr<VisionDataSourceAdapter>;
  [[nodiscard]] auto createTrackerDataSourceAdapter() const
    -> std::unique_ptr<TrackerDataSourceAdapter>;

private:
  rclcpp::Node & node_;
};

// Registry for managing multiple data sources
class DataSourceRegistry
{
public:
  explicit DataSourceRegistry();
  ~DataSourceRegistry() = default;

  // Registration methods
  auto registerBallDataSource(const std::string & name, std::unique_ptr<BallDataSource> source)
    -> void;
  auto registerRobotDataSource(const std::string & name, std::unique_ptr<RobotDataSource> source)
    -> void;
  auto registerCompositeDataSource(
    const std::string & name, std::unique_ptr<CompositeDataSource> source) -> void;

  // Access methods
  [[nodiscard]] auto getBallDataSource(const std::string & name) const
    -> std::shared_ptr<BallDataSource>;
  [[nodiscard]] auto getRobotDataSource(const std::string & name) const
    -> std::shared_ptr<RobotDataSource>;
  [[nodiscard]] auto getCompositeDataSource(const std::string & name) const
    -> std::shared_ptr<CompositeDataSource>;

  // Discovery methods
  [[nodiscard]] auto listBallDataSources() const -> std::vector<std::string>;
  [[nodiscard]] auto listRobotDataSources() const -> std::vector<std::string>;
  [[nodiscard]] auto listCompositeDataSources() const -> std::vector<std::string>;

  // Priority-based selection
  [[nodiscard]] auto selectBestBallDataSource() const -> std::shared_ptr<BallDataSource>;
  [[nodiscard]] auto selectBestRobotDataSource() const -> std::shared_ptr<RobotDataSource>;

  // Health monitoring
  auto performHealthCheck() -> void;
  [[nodiscard]] auto getHealthStatus() const -> std::string;

private:
  std::map<std::string, std::shared_ptr<BallDataSource>> ball_sources_;
  std::map<std::string, std::shared_ptr<RobotDataSource>> robot_sources_;
  std::map<std::string, std::shared_ptr<CompositeDataSource>> composite_sources_;

  mutable std::mutex registry_mutex_;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__DATA_SOURCE_ADAPTERS_HPP_
