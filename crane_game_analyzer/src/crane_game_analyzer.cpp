// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <vector>

#include "crane_game_analyzer/game_analyzer.hpp"
#include <rclcpp/rclcpp.hpp>

namespace crane
{
GameAnalyzerComponent::GameAnalyzerComponent(const rclcpp::NodeOptions & options)
: Node("crane_game_analyzer", options), ros_clock_(RCL_ROS_TIME)
{
  RCLCPP_INFO(get_logger(), "GameAnalyzer is constructed.");

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

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::GameAnalyzerComponent)
