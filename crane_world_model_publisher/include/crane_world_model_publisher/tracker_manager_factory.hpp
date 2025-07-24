// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__TRACKER_MANAGER_FACTORY_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__TRACKER_MANAGER_FACTORY_HPP_

#include <crane_world_model_publisher/ball_tracker.hpp>
#include <crane_world_model_publisher/robot_tracker.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace crane
{

struct TrackerManagerConfig
{
  double ball_outlier_threshold = 9.0;
  double ball_min_confidence = 0.3;
  double ball_max_age_seconds = 1.0;

  double robot_outlier_threshold = 9.0;
  double robot_min_confidence = 0.2;
  double robot_max_age_seconds = 2.0;

  bool enable_physics_validation = true;
  bool enable_multi_ball_tracking = false;  // 通常は単一ボール
};

/**
 * @brief TrackerManagerFactoryは、BallTrackerManagerとRobotTrackerManagerの
 * 生成と設定を管理するファクトリークラスです。
 *
 * 複数のコンポーネントが同じTrackerManagerインスタンスを共有できるように
 * シングルトンパターンまたは依存注入パターンをサポートします。
 */
class TrackerManagerFactory
{
public:
  explicit TrackerManagerFactory(rclcpp::Node & node);
  ~TrackerManagerFactory() = default;

  // Factory methods
  auto createBallTrackerManager() -> std::unique_ptr<BallTrackerManager>;
  auto createRobotTrackerManager() -> std::unique_ptr<RobotTrackerManager>;

  // Shared instance management (Singleton pattern for shared usage)
  auto getSharedBallTrackerManager() -> std::shared_ptr<BallTrackerManager>;
  auto getSharedRobotTrackerManager() -> std::shared_ptr<RobotTrackerManager>;

  // Configuration
  auto setTrackerConfig(const TrackerManagerConfig & config) -> void;
  [[nodiscard]] auto getTrackerConfig() const -> const TrackerManagerConfig &;

  // Parameter loading from ROS parameters
  auto loadConfigFromParameters() -> void;

private:
  rclcpp::Node & node_;
  TrackerManagerConfig config_;

  // Shared instances (lazy initialization)
  std::shared_ptr<BallTrackerManager> shared_ball_tracker_;
  std::shared_ptr<RobotTrackerManager> shared_robot_tracker_;

  auto initializeSharedInstances() -> void;
};

/**
 * @brief TrackerManagerContainerは、複数のTrackerManagerを統合管理するクラスです。
 *
 * VisionDataProcessorやWorldModelDataProviderから独立して、
 * TrackerManagerたちの生存期間とアクセスを管理します。
 */
class TrackerManagerContainer
{
public:
  explicit TrackerManagerContainer(rclcpp::Node & node);
  ~TrackerManagerContainer() = default;

  // Accessor methods
  [[nodiscard]] auto getBallTrackerManager() -> std::shared_ptr<BallTrackerManager>;
  [[nodiscard]] auto getRobotTrackerManager() -> std::shared_ptr<RobotTrackerManager>;

  // Configuration and initialization
  auto setTrackerConfig(const TrackerManagerConfig & config) -> void;
  auto initialize() -> void;

  // Integration with other components
  auto connectToVisionProcessor() -> void;
  auto connectToDataSourceManager() -> void;

  // Lifecycle management
  auto predict(double dt) -> void;
  auto removeOldTrackers() -> void;

private:
  rclcpp::Node & node_;
  std::unique_ptr<TrackerManagerFactory> factory_;

  std::shared_ptr<BallTrackerManager> ball_tracker_manager_;
  std::shared_ptr<RobotTrackerManager> robot_tracker_manager_;

  bool initialized_ = false;
};

/**
 * @brief TrackerServiceInterfaceは、TrackerManagerへの統一されたアクセスを提供します。
 *
 * 異なるコンポーネントが一貫したインターフェースでTrackerManagerにアクセスできます。
 */
class TrackerServiceInterface
{
public:
  virtual ~TrackerServiceInterface() = default;

  // Ball tracking services
  virtual auto processBallDetection(const Vector3 & position, const rclcpp::Time & timestamp)
    -> crane_msgs::msg::BallInfo = 0;
  virtual auto getBestBallTracker() -> std::shared_ptr<BallTracker> = 0;

  // Robot tracking services
  virtual auto processRobotDetection(
    uint8_t robot_id, RobotTrackerType type, const Vector3 & pose, const rclcpp::Time & timestamp)
    -> void = 0;
  virtual auto getRobotTracker(uint8_t robot_id, RobotTrackerType type)
    -> std::shared_ptr<RobotTracker> = 0;
  virtual auto getAllRobotInfo() -> std::vector<crane_msgs::msg::RobotInfo> = 0;

  // Feedback integration
  virtual auto updateRobotFeedback(
    uint8_t robot_id, const crane_msgs::msg::RobotFeedback & feedback) -> void = 0;
  virtual auto updateRobotCommand(uint8_t robot_id, const Vector2 & cmd_vel, double cmd_omega)
    -> void = 0;

  // System services
  virtual auto predict(double dt) -> void = 0;
  virtual auto removeOldTrackers() -> void = 0;
};

/**
 * @brief TrackerServiceImplementationは、TrackerServiceInterfaceの具体的な実装です。
 */
class TrackerServiceImplementation : public TrackerServiceInterface
{
public:
  explicit TrackerServiceImplementation(std::shared_ptr<TrackerManagerContainer> container);
  ~TrackerServiceImplementation() override = default;

  // Implement TrackerServiceInterface
  auto processBallDetection(const Vector3 & position, const rclcpp::Time & timestamp)
    -> crane_msgs::msg::BallInfo override;
  auto getBestBallTracker() -> std::shared_ptr<BallTracker> override;

  auto processRobotDetection(
    uint8_t robot_id, RobotTrackerType type, const Vector3 & pose, const rclcpp::Time & timestamp)
    -> void override;
  auto getRobotTracker(uint8_t robot_id, RobotTrackerType type)
    -> std::shared_ptr<RobotTracker> override;
  auto getAllRobotInfo() -> std::vector<crane_msgs::msg::RobotInfo> override;

  auto updateRobotFeedback(uint8_t robot_id, const crane_msgs::msg::RobotFeedback & feedback)
    -> void override;
  auto updateRobotCommand(uint8_t robot_id, const Vector2 & cmd_vel, double cmd_omega)
    -> void override;

  auto predict(double dt) -> void override;
  auto removeOldTrackers() -> void override;

private:
  std::shared_ptr<TrackerManagerContainer> container_;
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__TRACKER_MANAGER_FACTORY_HPP_
