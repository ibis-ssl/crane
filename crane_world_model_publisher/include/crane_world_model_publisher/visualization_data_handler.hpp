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

#include <robocup_ssl_msgs/ssl_vision_geometry.pb.h>
#include <robocup_ssl_msgs/ssl_vision_wrapper_tracked.pb.h>

#include <consai_visualizer_msgs/msg/objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>

namespace crane
{

using VisualizerObjects = consai_visualizer_msgs::msg::Objects;
using Referee = robocup_ssl_msgs::msg::Referee;

class VisualizationDataHandler
{
public:
  explicit VisualizationDataHandler(rclcpp::Node & node);
  ~VisualizationDataHandler() = default;

  void publish_vis_geometry(const SSL_GeometryData & geometry_data);
  void publish_vis_tracked(const TrackedFrame & tracked_frame);
  void publish_vis_referee(const Referee::SharedPtr msg);

private:
  rclcpp::Subscription<Referee>::SharedPtr sub_referee_;

  rclcpp::Publisher<VisualizerObjects>::SharedPtr pub_vis_objects_;
};

}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__VISUALIZATION_DATA_HANDLER_HPP_
