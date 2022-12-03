// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WAITER_PLANNER__WORLD_MODEL_PUBLISHER_HPP
#define CRANE_WAITER_PLANNER__WORLD_MODEL_PUBLISHER_HPP

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CRANE_EXPORT __attribute__((dllexport))
#define CRANE_IMPORT __attribute__((dllimport))
#else
#define CRANE_EXPORT __declspec(dllexport)
#define CRANE_IMPORT __declspec(dllimport)
#endif
#ifdef CRANE_BUILDING_DLL
#define CRANE_PUBLIC CRANE_EXPORT
#else
#define CRANE_PUBLIC CRANE_IMPORT
#endif
#define CRANE_PUBLIC_TYPE CRANE_PUBLIC
#define CRANE_LOCAL
#else
#define CRANE_EXPORT __attribute__((visibility("default")))
#define CRANE_IMPORT
#if __GNUC__ >= 4
#define CRANE_PUBLIC __attribute__((visibility("default")))
#define CRANE_LOCAL __attribute__((visibility("hidden")))
#else
#define CRANE_PUBLIC
#define CRANE_LOCAL
#endif
#define CRANE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "boost/range/adaptor/indexed.hpp"
#include "crane_msgs/msg/ball_info.hpp"
#include "crane_msgs/msg/robot_info.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"

namespace crane
{
enum class Color : uint8_t {
  BLUE,
  YELLOW,
};

class WorldModelPublisherComponent : public rclcpp::Node
{
public:
  CRANE_PUBLIC
  explicit WorldModelPublisherComponent(const rclcpp::NodeOptions &);

  void visionDetectionsCallback(const robocup_ssl_msgs::msg::TrackedFrame::SharedPtr);

  void visionGeometryCallback(const robocup_ssl_msgs::msg::GeometryData::SharedPtr);

private:
  void publishWorldModel();

  Color our_color;
  Color their_color;
  uint8_t max_id;
  static constexpr float DISAPPEARED_TIME_THRESH_ = 3.0f;
  double field_w_, field_h_;
  double goal_w_, goal_h_;
  double defense_area_w_, defense_area_h_;

  crane_msgs::msg::BallInfo ball_info_;
  std::vector<crane_msgs::msg::RobotInfo> robot_info_[2];
  rclcpp::Subscription<robocup_ssl_msgs::msg::TrackedFrame>::SharedPtr sub_vision_;
  rclcpp::Subscription<robocup_ssl_msgs::msg::GeometryData>::SharedPtr sub_geometry_;
  rclcpp::Publisher<crane_msgs::msg::WorldModel>::SharedPtr pub_world_model_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace crane
#endif  //CRANE_WAITER_PLANNER__WORLD_MODEL_PUBLISHER_HPP
