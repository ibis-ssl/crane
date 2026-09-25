// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_
#define CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_

#include <algorithm>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/game_analysis.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_visualization_interfaces/crane_visualizer_wrapper.hpp>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crane_game_analyzer/kick_event_detector.hpp"
#include "crane_game_analyzer/metrics/metric_engine.hpp"
#include "visibility_control.h"

namespace crane
{
struct BallIdleConfig
{
  rclcpp::Duration threshold_duration = rclcpp::Duration(5, 0);
  double move_distance_threshold_meter = 0.05;
};

struct RobotCollisionConfig
{
  double velocity_threshold = 1.0;  // m/s
  double distance_threshold = 0.2;  // m
  double time_window = 0.5;         // seconds
};

struct GameAnalyzerConfig
{
  BallIdleConfig ball_idle;
  RobotCollisionConfig robot_collision;
};

struct BallTouchInfo
{
  RobotIdentifier robot_id;
  double distance;
};

struct BallPositionStamped
{
  Point position;
  rclcpp::Time stamp;
};

struct RobotCollisionInfo
{
  RobotIdentifier attack_robot;
  RobotIdentifier attacked_robot;
  double relative_velocity;
};

class GameAnalyzerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit GameAnalyzerComponent(const rclcpp::NodeOptions & options);

private:
  auto getRobotCollisionInfo() -> std::optional<RobotCollisionInfo>
  {
    auto collision = detectCollision();

    if (collision) {
      visualizeCollision(*collision);
    }

    return collision;
  }

  auto detectCollision() -> std::optional<RobotCollisionInfo>
  {
    for (size_t i = 0; i < world_model->ours().robots.size(); ++i) {
      auto & our_robot = world_model->ours().robots[i];

      for (size_t j = 0; j < world_model->theirs().robots.size(); ++j) {
        auto & their_robot = world_model->theirs().robots[j];

        double distance = (our_robot->pose.pos - their_robot->pose.pos).norm();

        if (distance < config.robot_collision.distance_threshold) {
          Vector2 relative_velocity = our_robot->vel.linear - their_robot->vel.linear;
          double rel_vel_norm = relative_velocity.norm();

          if (rel_vel_norm > config.robot_collision.velocity_threshold) {
            // 速度ベクトルが互いに向かい合っているかチェック
            Vector2 direction = (their_robot->pose.pos - our_robot->pose.pos).normalized();
            double approach_factor = relative_velocity.normalized().dot(direction);

            // 正の値は互いに近づいていることを示す
            if (approach_factor > 0.5) {
              RobotCollisionInfo info;

              // 速度が大きい方を「攻撃側」と判定
              if (our_robot->vel.linear.norm() > their_robot->vel.linear.norm()) {
                info.attack_robot = RobotIdentifier{.is_ours = true, .id = our_robot->id};
                info.attacked_robot = RobotIdentifier{.is_ours = false, .id = their_robot->id};
              } else {
                info.attack_robot = RobotIdentifier{.is_ours = false, .id = their_robot->id};
                info.attacked_robot = RobotIdentifier{.is_ours = true, .id = our_robot->id};
              }

              info.relative_velocity = rel_vel_norm;
              return info;
            }
          }
        }
      }
    }

    return std::nullopt;
  }

  auto visualizeCollision(const RobotCollisionInfo & collision) const -> void
  {
    Point attack_pos, attacked_pos;

    if (collision.attack_robot.is_ours) {
      attack_pos = world_model->getOurRobot(collision.attack_robot.id)->pose.pos;
    } else {
      attack_pos = world_model->getTheirRobot(collision.attack_robot.id)->pose.pos;
    }

    if (collision.attacked_robot.is_ours) {
      attacked_pos = world_model->getOurRobot(collision.attacked_robot.id)->pose.pos;
    } else {
      attacked_pos = world_model->getTheirRobot(collision.attacked_robot.id)->pose.pos;
    }

    Point collision_point = (attack_pos + attacked_pos) * 0.5;

    visualizer->drawStyledCircle(collision_point, 0.15, "red", 0.3, "red", 1.0, 3);

    visualizer->line().start(attack_pos).end(attacked_pos).stroke("red").strokeWidth(2).build();

    std::string velocity_text = std::to_string(collision.relative_velocity).substr(0, 4) + " m/s";
    visualizer->drawCenteredLabel(collision_point + Vector2(0, 0.2), velocity_text, "red", 40);
  }

  WorldModelWrapper::UniquePtr world_model;

  GameAnalyzerConfig config;

  VisualizerMessageBuilder::SharedPtr visualizer;

  rclcpp::Publisher<crane_msgs::msg::GameAnalysis>::SharedPtr game_analysis_pub_;

  rclcpp::Publisher<crane_msgs::msg::KickPredictionTrace>::SharedPtr kick_prediction_trace_pub_;

  std::unique_ptr<metrics::MetricEngine> metric_engine_;
  std::deque<crane_msgs::msg::BallInfo> ball_history_;

  std::unique_ptr<KickEventDetector> kick_event_detector_;
  rclcpp::Subscription<crane_msgs::msg::RobotCommands>::SharedPtr sub_robot_commands_;
};
}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_
