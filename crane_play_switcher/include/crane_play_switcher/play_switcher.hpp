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

#ifndef CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_
#define CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_

#include "consai2r2_msgs/msg/decoded_referee.hpp"
#include "consai2r2_msgs/msg/vision_geometry.hpp"
#include "consai2r2_msgs/msg/vision_detection.hpp"
#include "crane_msgs/msg/in_play.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class PlaySwitcher : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PlaySwitcher(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<crane_msgs::msg::InPlay>::SharedPtr pub_switch_;
  rclcpp::Subscriber<consai2r2_msgs::msg::DecodedReferee>::SharedPtr sub_decoded_referee_;
  rclcpp::Subscriber<consai2r2_msgs::msg::VisionGeometry>::SharedPtr sub_vision_geometry_;
  rclcpp::Subscriber<consai2r2_msgs::msg::VisionDetection>::SharedPtr sub_vision_detection_;

  referee_callback(const consai2r2_msgs::msg::Referee::Sharedptr msg);
  vision_geometry_callback(const consai2r2_msgs::msg::VisionGeometry::Sharedptr msg);
  vision_detection_callback(const consai2r2_msgs::msg::VisionDetection::Sharedptr msg);

}
}  // namespace crane


#endif  // CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_
