// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crane_game_analyzer/game_analyzer.hpp"

namespace crane
{
GameAnalyzerComponent::GameAnalyzerComponent(const rclcpp::NodeOptions & options)
: Node("crane_game_analyzer", options),
  game_analysis_pub(create_publisher<crane_msgs::msg::GameAnalysis>("game_analysis", 10)),
  visualizer(std::make_unique<CraneVisualizerBuffer::MessageBuilder>("game_analyzer"))
{
  RCLCPP_INFO(get_logger(), "GameAnalyzer is constructed.");

  CraneVisualizerBuffer::activate(*this);

  world_model = std::make_unique<WorldModelWrapper>(*this);

  world_model->addCallback([&]() {
    kick_event_detector.update(*world_model, visualizer);
    crane_msgs::msg::GameAnalysis game_analysis_msg;
    updateBallPossession(game_analysis_msg.ball);
    if (auto kick = kick_event_detector.getOnGoingKick(); kick.has_value()) {
      game_analysis_msg.ongoing_kick.push_back(*kick);
    }

    game_analysis_pub->publish(game_analysis_msg);
    auto robot_collision_info = getRobotCollisionInfo();

    if (robot_collision_info) {
      //          robot_collision_info->attack_robot.robot_id
      RCLCPP_INFO(
        get_logger(), "Collision Detected : ( %d, %d ) , %f [m/s]",
        robot_collision_info->attack_robot.robot_id, robot_collision_info->attacked_robot.robot_id,
        robot_collision_info->relative_velocity);
    }
    visualizer->flush();
    CraneVisualizerBuffer::publish();
  });
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::GameAnalyzerComponent)
