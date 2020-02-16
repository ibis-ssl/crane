// Copyright (c) 2019 ibis-ssl
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

#include "crane_play_switcher/play_switcher.hpp"

#include "rclcpp/rclcpp.hpp"

namespace crane
{
PlaySwitcher::PlaySwitcher(const rclcpp::NodeOptions & options)
: Node("crane_play_switcher", options)
{
  RCLCPP_INFO(this->get_logger(), "PlaySwitcher is constructed.");
  auto referee_callback =
    [this](const consai2r2_msgs::msg::DecodedReferee::SharedPtr msg) -> void
    {
      this->referee_callback(msg);
    };
  auto vision_geometry_callback =
    [this](const consai2r2_msgs::msg::VisionGeometry::SharedPtr msg) -> void
    {
      this->vision_geometry_callback(msg);
    };
  auto vision_detections_callback =
    [this](const consai2r2_msgs::msg::VisionDetections::SharedPtr msg) -> void
    {
      this->vision_detections_callback(msg);
    };
  pub_play_situation_ = this->create_publisher<crane_msgs::msg::PlaySituation>("", 10);
  sub_decoded_referee_ = this->create_subscription<consai2r2_msgs::msg::DecodedReferee>("", 10,
      referee_callback);
  sub_vision_geometry_ = this->create_subscription<consai2r2_msgs::msg::VisionGeometry>("", 5,
      vision_geometry_callback);
  sub_vision_detections_ = this->create_subscription<consai2r2_msgs::msg::VisionDetections>("", 5,
      vision_detections_callback);
}

void PlaySwitcher::referee_callback(const consai2r2_msgs::msg::DecodedReferee::SharedPtr msg)
{
    play_situation_msg_.referee_id = msg->referee_id;
    play_situation_msg_.referee_text= msg->referee_text;
    play_situation_msg_.is_inplay = msg->is_inplay;
    play_situation_msg_.placement_position = msg->placement_position;
}

void PlaySwitcher::vision_geometry_callback(const consai2r2_msgs::msg::VisionGeometry::SharedPtr msg)
{
    // play_situation_msg_.world_model.header = msg->placement_position;
}

void PlaySwitcher::vision_detections_callback(
  const consai2r2_msgs::msg::VisionDetections::SharedPtr msg)
{
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crane::PlaySwitcher)
