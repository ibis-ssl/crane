// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "game_analyzer.hpp"

namespace crane
{
GameAnalyzerComponent::GameAnalyzerComponent(const rclcpp::NodeOptions & options)
: Node("crane_game_analyzer", options)
{
  RCLCPP_INFO(get_logger(), "GameAnalyzer is constructed.");

  world_model = std::make_unique<WorldModelWrapper>(*this);

  world_model->addCallback([&]() {
    crane_msgs::msg::GameAnalysis game_analysis_msg;
    bool is_ball_idle = getBallIdle();
    updateBallPossesion(game_analysis_msg.ball);
    auto robot_collision_info = getRobotCollisionInfo();

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
