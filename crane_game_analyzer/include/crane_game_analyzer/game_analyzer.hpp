// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_
#define CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_

#include <algorithm>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/game_analysis.hpp>
#include <crane_msgs/msg/game_event.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>

#include "crane_game_analyzer/event_memory.hpp"
#include "crane_game_analyzer/ronar_event_detector.hpp"
#include "crane_game_analyzer/threat_evaluator.hpp"
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

struct RobotPositionStamped
{
  uint8_t id;
  bool is_ours;
  Point position;
  Point velocity;  // Point型に変更（Velocity2D型のlinearメンバーと同等）
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
  auto getBallIdle() -> bool
  {
    BallPositionStamped record;
    record.position = world_model->ball().pos;
    record.stamp = now();
    static std::deque<BallPositionStamped> ball_records;
    ball_records.push_front(record);

    auto latest_time = ball_records.front().stamp;
    auto latest_position = ball_records.front().position;
    // 一定時間以上前の履歴を削除
    std::erase_if(ball_records, [&](auto & ball_record) {
      return (latest_time - ball_record.stamp) > config.ball_idle.threshold_duration * 2;
    });

    // ボール履歴（新しいほど，indexが若い）のチェックして，ボールがストップしているかを確認
    return not std::ranges::any_of(ball_records, [&](const auto & ball_record) {
      bool distance_cond = (latest_position - ball_record.position).norm() <
                           config.ball_idle.move_distance_threshold_meter;
      bool time_cond = (latest_time - ball_record.stamp) < config.ball_idle.threshold_duration;
      return distance_cond && time_cond;
    });
  }

  auto getRobotCollisionInfo() -> std::optional<RobotCollisionInfo>
  {
    // 現在のロボット状態を記録
    recordCurrentRobotStates();

    // 衝突検知アルゴリズム
    auto collision = detectCollision();

    // 衝突を検出した場合、可視化
    if (collision) {
      visualizeCollision(*collision);
    }

    return collision;
  }

  auto recordCurrentRobotStates() -> void
  {
    auto current_time = now();

    // 自チームのロボット位置を記録
    for (const auto & robot : world_model->ours().getAvailableRobots()) {
      RobotPositionStamped record;
      record.id = robot->id;
      record.is_ours = true;
      record.position = robot->pose.pos;
      record.velocity = robot->vel.linear;
      record.stamp = current_time;
      robot_records_.push_front(record);
    }

    // 相手チームのロボット位置を記録
    for (const auto & robot : world_model->theirs().getAvailableRobots()) {
      RobotPositionStamped record;
      record.id = robot->id;
      record.is_ours = false;
      record.position = robot->pose.pos;
      record.velocity = robot->vel.linear;
      record.stamp = current_time;
      robot_records_.push_front(record);
    }

    // 古い記録を削除
    auto time_threshold =
      current_time - rclcpp::Duration::from_seconds(config.robot_collision.time_window * 2);
    std::erase_if(
      robot_records_, [&](const auto & record) { return record.stamp < time_threshold; });
  }

  auto detectCollision() -> std::optional<RobotCollisionInfo>
  {
    // 全てのロボットペアをチェック
    for (size_t i = 0; i < world_model->ours().robots.size(); ++i) {
      auto & our_robot = world_model->ours().robots[i];

      for (size_t j = 0; j < world_model->theirs().robots.size(); ++j) {
        auto & their_robot = world_model->theirs().robots[j];

        // ロボット間の距離
        double distance = (our_robot->pose.pos - their_robot->pose.pos).norm();

        // 距離が閾値以下ならば衝突の可能性
        if (distance < config.robot_collision.distance_threshold) {
          // 相対速度を計算
          Vector2 relative_velocity = our_robot->vel.linear - their_robot->vel.linear;
          double rel_vel_norm = relative_velocity.norm();

          // 相対速度が閾値以上ならば衝突と判定
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
    // 衝突ロボットの位置を取得
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

    // 衝突点を可視化（中間点）
    Point collision_point = (attack_pos + attacked_pos) * 0.5;

    // 衝突箇所に赤い円を描画
    visualizer->drawStyledCircle(collision_point, 0.15, "red", 0.3, "red", 1.0, 3);

    // 衝突ロボット間に線を描画
    visualizer->line().start(attack_pos).end(attacked_pos).stroke("red").strokeWidth(2).build();

    // 速度表示
    std::string velocity_text = std::to_string(collision.relative_velocity).substr(0, 4) + " m/s";
    visualizer->drawCenteredLabel(collision_point + Vector2(0, 0.2), velocity_text, "red", 40);
  }

  WorldModelWrapper::UniquePtr world_model;

  GameAnalyzerConfig config;

  VisualizerMessageBuilder::SharedPtr visualizer;

  // ロボット位置の履歴
  std::deque<RobotPositionStamped> robot_records_;

  // 脅威評価システム
  ThreatEvaluator threat_evaluator_;
  rclcpp::Publisher<crane_msgs::msg::GameAnalysis>::SharedPtr game_analysis_pub_;

  // RONARイベント検出システム
  std::unique_ptr<RonarEventDetector> ronar_event_detector_;
  rclcpp::Publisher<crane_msgs::msg::RonarEvent>::SharedPtr ronar_events_pub_;

  // イベントメモリシステム
  std::unique_ptr<EventMemory> event_memory_;

  // PlaySituation 購読
  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_sub_;
  std::optional<uint8_t> last_play_situation_command_;

  // GameEvent (autoref) 購読
  rclcpp::Subscription<crane_msgs::msg::GameEvent>::SharedPtr game_event_sub_;

  // チーム名
  std::string our_team_name_;
  std::string their_team_name_;

  auto evaluateThreats() -> crane_msgs::msg::GameAnalysis;
  auto detectAndPublishRonarEvents() -> void;

  // PlaySituation イベント処理
  auto onPlaySituationChanged(const crane_msgs::msg::PlaySituation & msg) -> void;
  auto createPlaySituationEvent(uint8_t event_type, const crane_msgs::msg::PlaySituation & msg)
    -> crane_msgs::msg::RonarEvent;

  // GameEvent (autoref) イベント処理
  auto onGameEvent(const crane_msgs::msg::GameEvent & msg) -> void;
};
}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_
