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

#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "crane_game_analyzer/game_analyzer.hpp"

namespace crane
{
GameAnalyzer::GameAnalyzer(const rclcpp::NodeOptions& options)
  : Node("crane_game_analyzer", options), ros_clock_(RCL_ROS_TIME)
{
  RCLCPP_INFO(this->get_logger(), "GameAnalyzer is constructed.");

  auto world_model_callback = [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void {
    this->world_model_callback(*msg);
  };
  // FIXME トピック名を合わせる
  sub_world_model_ =
      create_subscription<crane_msgs::msg::WorldModel>("crane_world_observer/world_model", 10, world_model_callback);
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crane::GameAnalyzer)
