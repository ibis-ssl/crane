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

#include "crane_game_analyzer/game_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace crane
{
GameAnalyzer::GameAnalyzer(const rclcpp::NodeOptions & options)
: Node("crane_game_analyzer", options), ros_clock_(RCL_ROS_TIME)
{
  RCLCPP_INFO(this->get_logger(), "GameAnalyzer is constructed.");

  world_model_ = std::make_shared<WorldModelWrapper>(*this);
  world_model_->addCallback([this](void) -> void {
    bool is_ball_idle = getBallIdle();
    auto ball_touch_info = getBallTouchInfo();
    auto robot_collision_info = getRobotCollisionInfo();

    static BallTouchInfo last_touch;
    if (ball_touch_info) {
      last_touch = *ball_touch_info;
    }

    if (robot_collision_info) {
      //          robot_collision_info->attack_robot.robot_id
      RCLCPP_INFO(
        get_logger(), "Collision Detected : ( %d, %d ) , %f [m/s]",
        robot_collision_info->attack_robot.robot_id, robot_collision_info->attacked_robot.robot_id,
        robot_collision_info->relative_velocity);
    }
  });
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crane::GameAnalyzer)
