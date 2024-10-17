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

#include "consai_vision_tracker/tracker_component.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "robocup_ssl_msgs/msg/robot_id.hpp"

namespace consai_vision_tracker
{
using RobotId = robocup_ssl_msgs::msg::RobotId;
using std::placeholders::_1;

VisionVisualizer::VisionVisualizer(const rclcpp::NodeOptions & options) : Node("tracker", options)
{
  std::cout << "VisionVisualizer" << std::endl;
  vis_data_handler_ = std::make_shared<VisualizationDataHandler>(*this);

  sub_detection_tracked = create_subscription<TrackedFrame>(
    "detection", 10, std::bind(&VisionVisualizer::callback_detection_tracked, this, _1));

  sub_geometry_ = create_subscription<GeometryData>(
    "geometry", 10, std::bind(&VisionVisualizer::callback_geometry, this, _1));
}

void VisionVisualizer::callback_detection_tracked(TrackedFrame::UniquePtr msg)
{
  vis_data_handler_->publish_vis_tracked(std::move(msg));
}

void VisionVisualizer::callback_geometry(const GeometryData::SharedPtr msg)
{
  vis_data_handler_->publish_vis_geometry(msg);
}

}  // namespace consai_vision_tracker
