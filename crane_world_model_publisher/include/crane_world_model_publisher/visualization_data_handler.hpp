// Copyright 2023 Roots
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

#ifndef CRANE_WORLD_MODEL_PUBLISHER__VISUALIZATION_DATA_HANDLER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__VISUALIZATION_DATA_HANDLER_HPP_

#include <robocup_ssl_msgs/ssl_vision_detection.pb.h>
#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>

namespace crane
{
using Referee = robocup_ssl_msgs::msg::Referee;

class VisualizationDataHandler
{
public:
  explicit VisualizationDataHandler(rclcpp::Node & node);
  ~VisualizationDataHandler() = default;

  auto publish_vis_geometry(
    const SSL_GeometryData & geometry_data, const bool half_court_practice_mode) -> void;

  auto publish_vis_detection(
    const SSL_DetectionFrame & detection, const bool half_court_practice_mode) -> void;

  auto publish_vis_tracked(const WorldModelWrapper::SharedPtr &) -> void;

  auto publish_vis_referee(const Referee & msg, double field_width, double field_height) -> void;

private:
  crane::VisualizerMessageBuilder::SharedPtr visualizer_geometry;

  crane::VisualizerMessageBuilder::SharedPtr visualizer_vision;

  crane::VisualizerMessageBuilder::SharedPtr visualizer_tracked;

  crane::VisualizerMessageBuilder::SharedPtr visualizer_referee;

  double ball_x;

  double ball_y;
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISUALIZATION_DATA_HANDLER_HPP_
