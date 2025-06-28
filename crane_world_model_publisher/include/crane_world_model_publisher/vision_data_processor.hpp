// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__VISION_DATA_PROCESSOR_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__VISION_DATA_PROCESSOR_HPP_

#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper.pb.h>

#include <crane_comm/multicast.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <crane_world_model_publisher/ball_tracker.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace crane
{
class VisionDataProcessor
{
public:
  explicit VisionDataProcessor(rclcpp::Node & node);

  ~VisionDataProcessor() = default;

  auto processVisionPackets() -> void;

  [[nodiscard]] auto hasVisionUpdated() const -> bool { return has_vision_updated_; }

  [[nodiscard]] auto getFieldWidth() const -> double { return field_width_; }

  [[nodiscard]] auto getFieldHeight() const -> double { return field_height_; }

  [[nodiscard]] auto getGoalWidth() const -> double { return goal_width_; }

  [[nodiscard]] auto getGoalHeight() const -> double { return goal_height_; }

  [[nodiscard]] auto getPenaltyAreaWidth() const -> double { return penalty_area_width_; }

  [[nodiscard]] auto getPenaltyAreaHeight() const -> double { return penalty_area_height_; }

  [[nodiscard]] auto getBallInfo() const -> const crane_msgs::msg::BallInfo & { return ball_info_; }

  [[nodiscard]] auto getRobotInfo(int team_index) const
    -> const std::vector<crane_msgs::msg::RobotInfo> &
  {
    return robot_info_[team_index];
  }

  // Vision パケットタイムスタンプ取得
  [[nodiscard]] auto getLastVisionTCapture() const -> double { return last_t_capture_; }
  [[nodiscard]] auto getLastVisionTSent() const -> double { return last_t_sent_; }

  auto setVisualizationHandler(std::function<void(const SSL_GeometryData &, bool)> handler) -> void
  {
    geometry_vis_handler_ = handler;
  }

  auto setGeometryUpdateHandler(std::function<void()> handler) -> void
  {
    geometry_update_handler_ = handler;
  }

private:
  rclcpp::Node & node_;

  std::unique_ptr<multicast::MulticastReceiver> vision_receiver_;

  bool has_vision_updated_ = false;

  rclcpp::Time last_ball_detect_time_;

  double field_width_ = 0.0;
  double field_height_ = 0.0;
  double goal_width_ = 0.0;
  double goal_height_ = 0.0;
  double penalty_area_width_ = 0.0;
  double penalty_area_height_ = 0.0;

  crane_msgs::msg::BallInfo ball_info_;
  std::vector<crane_msgs::msg::RobotInfo> robot_info_[2];

  // Visionパケットタイムスタンプ
  double last_t_capture_ = 0.0;
  double last_t_sent_ = 0.0;

  std::unique_ptr<BallTrackerManager> ball_tracker_manager_;
  rclcpp::Time last_prediction_time_;

  std::function<void(const SSL_GeometryData &, bool)> geometry_vis_handler_;

  std::function<void()> geometry_update_handler_;

  auto visionGeometryCallback(const SSL_GeometryData & geometry_data) -> void;

  auto visionDetectionCallback(const SSL_DetectionFrame & detection_frame) -> void;

  enum class Color { BLUE, YELLOW };
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISION_DATA_PROCESSOR_HPP_
