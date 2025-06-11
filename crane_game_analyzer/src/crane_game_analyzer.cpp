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
  visualizer(std::make_shared<VisualizerMessageBuilder>("game_analyzer"))
{
  RCLCPP_INFO(get_logger(), "GameAnalyzer is constructed.");

  // パラメータの設定
  declare_parameter("ball_idle.threshold_duration", 5.0);
  declare_parameter("ball_idle.move_distance_threshold_meter", 0.05);
  declare_parameter("ball_possession.threshold_meter", 0.05);
  declare_parameter("robot_collision.velocity_threshold", 1.0);
  declare_parameter("robot_collision.distance_threshold", 0.2);
  declare_parameter("robot_collision.time_window", 0.5);

  // パラメータの読み込み
  config.ball_idle.threshold_duration =
    rclcpp::Duration::from_seconds(get_parameter("ball_idle.threshold_duration").as_double());
  config.ball_idle.move_distance_threshold_meter =
    get_parameter("ball_idle.move_distance_threshold_meter").as_double();
  config.ball_possession.threshold_meter =
    get_parameter("ball_possession.threshold_meter").as_double();
  config.robot_collision.velocity_threshold =
    get_parameter("robot_collision.velocity_threshold").as_double();
  config.robot_collision.distance_threshold =
    get_parameter("robot_collision.distance_threshold").as_double();
  config.robot_collision.time_window = get_parameter("robot_collision.time_window").as_double();

  RCLCPP_INFO(get_logger(), "Robot collision detection parameters:");
  RCLCPP_INFO(
    get_logger(), "  - Velocity threshold: %.2f m/s", config.robot_collision.velocity_threshold);
  RCLCPP_INFO(
    get_logger(), "  - Distance threshold: %.2f m", config.robot_collision.distance_threshold);
  RCLCPP_INFO(get_logger(), "  - Time window: %.2f s", config.robot_collision.time_window);

  CraneVisualizerBuffer::activate(*this);

  world_model = std::make_unique<WorldModelWrapper>(*this);

  world_model->addCallback([&]() {
    auto robot_collision_info = getRobotCollisionInfo();

    if (robot_collision_info) {
      //          robot_collision_info->attack_robot.robot_id
      RCLCPP_INFO(
        get_logger(), "Collision Detected : ( %d, %d ) , %f [m/s]",
        robot_collision_info->attack_robot.id, robot_collision_info->attacked_robot.id,
        robot_collision_info->relative_velocity);
    }
    visualizer->flush();
    CraneVisualizerBuffer::publish();
  });
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::GameAnalyzerComponent)
