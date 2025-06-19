// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__TRACKER_DATA_PROCESSOR_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__TRACKER_DATA_PROCESSOR_HPP_

#include <robocup_ssl_msgs/ssl_vision_detection_tracked.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>

#include <Eigen/Dense>
#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/geometry_operations.hpp>
#include <crane_basics/multicast.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/robot_id.hpp>
#include <string>
#include <vector>

namespace crane
{
class TrackerDataProcessor
{
public:
  explicit TrackerDataProcessor(rclcpp::Node & node);

  ~TrackerDataProcessor() = default;

  auto processTrackerPackets() -> void;

  [[nodiscard]] auto hasTrackerUpdated() const -> bool { return has_tracker_updated_; }

  [[nodiscard]] auto getBallInfo() const -> const crane_msgs::msg::BallInfo & { return ball_info_; }

  [[nodiscard]] auto getRobotInfo(int team_index) const
    -> const std::vector<crane_msgs::msg::RobotInfo> &
  {
    return robot_info_[team_index];
  }

  auto setAreaMask(const Box & area) -> void { area_mask_ = area; }

  auto setTransformMatrix(const Eigen::Matrix3d & matrix) -> void { transform_matrix_ = matrix; }

private:
  rclcpp::Node & node_;

  std::unique_ptr<multicast::MulticastReceiver> tracker_receiver_;

  bool has_tracker_updated_ = false;

  crane_msgs::msg::BallInfo ball_info_;
  std::vector<crane_msgs::msg::RobotInfo> robot_info_[2];

  Box area_mask_;
  Eigen::Matrix3d transform_matrix_ = Eigen::Matrix3d::Identity();

  auto trackerCallback(const TrackedFrame & tracked_frame) -> void;

  enum class Color { BLUE, YELLOW };
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__TRACKER_DATA_PROCESSOR_HPP_
