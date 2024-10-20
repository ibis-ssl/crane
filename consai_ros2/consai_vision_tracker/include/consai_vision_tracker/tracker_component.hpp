// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONSAI_VISION_TRACKER__TRACKER_COMPONENT_HPP_
#define CONSAI_VISION_TRACKER__TRACKER_COMPONENT_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/geometry_data.hpp>
#include <robocup_ssl_msgs/msg/tracked_frame.hpp>
#include <vector>

#include "visibility_control.h"
#include "visualization_data_handler.hpp"

namespace consai_vision_tracker
{
using GeometryData = robocup_ssl_msgs::msg::GeometryData;

using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;

class VisionVisualizer : public rclcpp::Node
{
public:
  CONSAI_VISION_TRACKER_PUBLIC
  explicit VisionVisualizer(const rclcpp::NodeOptions & options);

private:
  void callback_detection_tracked(TrackedFrame::UniquePtr msg);

  void callback_geometry(const GeometryData::SharedPtr msg);

  rclcpp::Subscription<TrackedFrame>::SharedPtr sub_detection_tracked;

  rclcpp::Subscription<GeometryData>::SharedPtr sub_geometry_;

  std::shared_ptr<VisualizationDataHandler> vis_data_handler_;
};

}  // namespace consai_vision_tracker

#endif  // CONSAI_VISION_TRACKER__TRACKER_COMPONENT_HPP_
