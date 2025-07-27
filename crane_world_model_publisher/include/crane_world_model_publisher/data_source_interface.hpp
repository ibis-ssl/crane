// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__DATA_SOURCE_INTERFACE_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__DATA_SOURCE_INTERFACE_HPP_

#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace crane
{
enum class DataSourcePriority {
  PRIMARY = 0,    // Highest priority (e.g., Vision with EKF)
  SECONDARY = 1,  // Medium priority (e.g., External tracker)
  FALLBACK = 2    // Lowest priority (e.g., Last known state)
};

enum class DataSourceCapability {
  BALL_TRACKING = 1 << 0,       // Can provide ball position/velocity data
  ROBOT_TRACKING = 1 << 1,      // Can provide robot position/velocity data
  FIELD_GEOMETRY = 1 << 2,      // Can provide field dimensions
  REAL_TIME_TRACKING = 1 << 3,  // Provides real-time tracking capabilities
  HISTORICAL_DATA = 1 << 4,     // Can provide historical position data
  HIGH_ACCURACY = 1 << 5        // Provides high-accuracy tracking
};

struct DataSourceMetadata
{
  std::string name;
  std::string description;
  DataSourcePriority priority;
  uint32_t capabilities;  // Bitfield of DataSourceCapability values
  double update_frequency_hz;
  double accuracy_estimate_meters;
  rclcpp::Time last_update_time;
  bool is_active;
};

class DataSourceBase
{
public:
  virtual ~DataSourceBase() = default;

  // Core interface methods
  [[nodiscard]] virtual auto getMetadata() const -> const DataSourceMetadata & = 0;
  [[nodiscard]] virtual auto isAvailable() const -> bool = 0;
  [[nodiscard]] virtual auto hasRecentUpdate(double max_age_seconds = 1.0) const -> bool = 0;

  // Data source management
  virtual auto activate() -> void = 0;
  virtual auto deactivate() -> void = 0;
  virtual auto reset() -> void = 0;

  // Quality assessment
  [[nodiscard]] virtual auto getQualityScore() const -> double = 0;  // [0.0, 1.0]
  [[nodiscard]] virtual auto getLatencyMilliseconds() const -> double = 0;

protected:
  DataSourceMetadata metadata_;
};

class BallDataSource : public virtual DataSourceBase
{
public:
  virtual ~BallDataSource() = default;

  // Ball-specific data access
  [[nodiscard]] virtual auto getBallInfo() const -> crane_msgs::msg::BallInfo = 0;
  [[nodiscard]] virtual auto hasBallUpdate() const -> bool = 0;

  // Advanced ball tracking features
  [[nodiscard]] virtual auto getBallConfidence() const -> double = 0;  // [0.0, 1.0]
  [[nodiscard]] virtual auto getBallTrackingState() const -> std::string = 0;

  // Optional: Historical ball data
  [[nodiscard]] virtual auto getBallHistory(size_t max_points = 100) const
    -> std::vector<crane_msgs::msg::BallInfo>
  {
    // Default implementation returns empty history
    (void)max_points;
    return {};
  }
};

class RobotDataSource : public virtual DataSourceBase
{
public:
  virtual ~RobotDataSource() = default;

  // Robot-specific data access
  [[nodiscard]] virtual auto getRobotInfo(int team_index) const
    -> std::vector<crane_msgs::msg::RobotInfo> = 0;
  [[nodiscard]] virtual auto hasRobotUpdate() const -> bool = 0;

  // Team management
  [[nodiscard]] virtual auto getTeamCount() const -> int { return 2; }  // Blue and Yellow
  [[nodiscard]] virtual auto getMaxRobotsPerTeam() const -> int { return 16; }

  // Advanced robot tracking features
  [[nodiscard]] virtual auto getRobotConfidence(int team_index, uint8_t robot_id) const
    -> double = 0;  // [0.0, 1.0]
  [[nodiscard]] virtual auto getRobotTrackingState(int team_index, uint8_t robot_id) const
    -> std::string = 0;

  // Optional: Historical robot data
  [[nodiscard]] virtual auto getRobotHistory(
    int team_index, uint8_t robot_id, size_t max_points = 100) const
    -> std::vector<crane_msgs::msg::RobotInfo>
  {
    // Default implementation returns empty history
    (void)team_index;
    (void)robot_id;
    (void)max_points;
    return {};
  }
};

class FieldGeometryDataSource : public virtual DataSourceBase
{
public:
  virtual ~FieldGeometryDataSource() = default;

  // Field geometry data access
  [[nodiscard]] virtual auto getFieldWidth() const -> double = 0;
  [[nodiscard]] virtual auto getFieldHeight() const -> double = 0;
  [[nodiscard]] virtual auto getGoalWidth() const -> double = 0;
  [[nodiscard]] virtual auto getGoalHeight() const -> double = 0;
  [[nodiscard]] virtual auto getPenaltyAreaWidth() const -> double = 0;
  [[nodiscard]] virtual auto getPenaltyAreaHeight() const -> double = 0;
  [[nodiscard]] virtual auto hasGeometryUpdate() const -> bool = 0;
};

// Composite interface for data sources that provide multiple types of data
class CompositeDataSource : public BallDataSource,
                            public RobotDataSource,
                            public FieldGeometryDataSource
{
public:
  virtual ~CompositeDataSource() = default;

  // Additional methods for composite sources
  [[nodiscard]] virtual auto getSupportedDataTypes() const -> uint32_t = 0;  // Capability bitfield
  [[nodiscard]] virtual auto canReplaceDataSource(const DataSourceBase & other) const -> bool = 0;
};

// Factory interface for creating data sources
class DataSourceFactory
{
public:
  virtual ~DataSourceFactory() = default;

  // Factory methods
  [[nodiscard]] virtual auto createBallDataSource(const std::string & config) const
    -> std::unique_ptr<BallDataSource> = 0;
  [[nodiscard]] virtual auto createRobotDataSource(const std::string & config) const
    -> std::unique_ptr<RobotDataSource> = 0;
  [[nodiscard]] virtual auto createCompositeDataSource(const std::string & config) const
    -> std::unique_ptr<CompositeDataSource> = 0;

  // Discovery methods
  [[nodiscard]] virtual auto listAvailableDataSources() const -> std::vector<std::string> = 0;
  [[nodiscard]] virtual auto validateConfiguration(const std::string & config) const -> bool = 0;
};

// Helper functions for capability checking
inline auto hasCapability(uint32_t capabilities, DataSourceCapability capability) -> bool
{
  return (capabilities & static_cast<uint32_t>(capability)) != 0;
}

inline auto addCapability(uint32_t & capabilities, DataSourceCapability capability) -> void
{
  capabilities |= static_cast<uint32_t>(capability);
}

inline auto removeCapability(uint32_t & capabilities, DataSourceCapability capability) -> void
{
  capabilities &= ~static_cast<uint32_t>(capability);
}

// Utility class for comparing data source priorities
class DataSourceComparator
{
public:
  auto operator()(const DataSourceBase & a, const DataSourceBase & b) const -> bool
  {
    const auto & meta_a = a.getMetadata();
    const auto & meta_b = b.getMetadata();

    // Primary comparison: priority (lower enum value = higher priority)
    if (meta_a.priority != meta_b.priority) {
      return meta_a.priority < meta_b.priority;
    }

    // Secondary comparison: quality score (higher is better)
    double quality_a = a.getQualityScore();
    double quality_b = b.getQualityScore();
    if (std::abs(quality_a - quality_b) > 0.01) {  // Avoid floating point equality
      return quality_a > quality_b;
    }

    // Tertiary comparison: latency (lower is better)
    return a.getLatencyMilliseconds() < b.getLatencyMilliseconds();
  }
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__DATA_SOURCE_INTERFACE_HPP_
