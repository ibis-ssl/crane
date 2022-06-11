// Copyright (c) 2022 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_WAITER_PLANNER__WORLD_MODEL_PUBLISHER_HPP
#define CRANE_WAITER_PLANNER__WORLD_MODEL_PUBLISHER_HPP

// THE SOFTWARE.

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
  WorldModelPublisherComponent(const rclcpp::NodeOptions &);

  void visionDetectionsCallback(const robocup_ssl_msgs::msg::TrackedFrame::SharedPtr);

  void visionGeometryCallback(const robocup_ssl_msgs::msg::GeometryData::SharedPtr);

private:
  void publishWorldModel();

  Color our_color;
  Color their_color;
  uint8_t max_id;
  static constexpr float DISAPPEARED_TIME_THRESH_ = 3.0f;
  float field_w_, field_h_;

  crane_msgs::msg::BallInfo ball_info_;
  std::vector<crane_msgs::msg::RobotInfo> robot_info_[2];
  rclcpp::Subscription<robocup_ssl_msgs::msg::TrackedFrame>::SharedPtr sub_vision_;
  rclcpp::Subscription<robocup_ssl_msgs::msg::GeometryData>::SharedPtr sub_geometry_;
  rclcpp::Publisher<crane_msgs::msg::WorldModel>::SharedPtr pub_world_model_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace crane
#endif  //CRANE_WAITER_PLANNER__WORLD_MODEL_PUBLISHER_HPP
