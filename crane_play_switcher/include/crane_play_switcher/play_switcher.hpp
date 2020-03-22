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
#include "crane_msgs/msg/world_model.hpp"
#include "crane_play_switcher/visibility_control.h"
#include "crane_msgs/msg/play_situation.hpp"
#include "crane_msgs/msg/in_play_situation.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class PlaySwitcher : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PlaySwitcher(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<crane_msgs::msg::PlaySituation>::SharedPtr pub_play_situation_;
  rclcpp::Subscription<consai2r2_msgs::msg::DecodedReferee>::SharedPtr sub_decoded_referee_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;

  void referee_callback(const consai2r2_msgs::msg::DecodedReferee::SharedPtr msg);
  void world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg);

  crane_msgs::msg::PlaySituation play_situation_msg_;
};
}  // namespace crane


#endif  // CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_
