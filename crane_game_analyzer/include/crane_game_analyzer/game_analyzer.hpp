// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_
#define CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "crane_game_analyzer/visibility_control.h"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msgs/msg/game_analysis.hpp"


namespace crane
{
struct BallIdleConfig
{
  rclcpp::Duration threshold_duration = rclcpp::Duration(5, 0);
  double move_distance_threshold_meter = 0.05;
};

struct BallPossesionConfig
{
  double threshold_meter = 0.05;
};
struct GameAnalyzerConfig
{
  BallIdleConfig ball_idle;
  BallPossesionConfig ball_possesion;
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
  void updateBallPossesion(crane_msgs::msg::BallAnalysis & analysis)
  {
    auto & ours = world_model_->ours.robots;
    auto & theirs = world_model_->theirs.robots;
    auto & ball_pos = world_model_->ball.pos;
    auto get_nearest_ball_robot = [&](std::vector<RobotInfo::SharedPtr> & robots) {
      return *std::min_element(robots.begin(), robots.end(), [ball_pos](auto & a, auto & b) {
        return (a->pose.pos - ball_pos).squaredNorm() < (b->pose.pos - ball_pos).squaredNorm();
      });
    };

    auto nearest_ours = get_nearest_ball_robot(ours);
    auto nearest_theirs = get_nearest_ball_robot(theirs);

    analysis.nearest_to_ball_robot_id_ours = nearest_ours->id;
    analysis.nearest_to_ball_robot_id_theirs  = nearest_theirs->id;

    double ours_distance = (nearest_ours->pose.pos - ball_pos).norm();
    double theirs_distance = (nearest_theirs->pose.pos - ball_pos).norm();

    const auto & threshold = config_.ball_possesion.threshold_meter;
    analysis.ball_possession_ours = (ours_distance < threshold);
    analysis.ball_possession_theirs = (theirs_distance < threshold);

  }

  bool getBallIdle()
  {
    BallPositionStamped record;
    record.position = world_model_->ball.pos;
    record.stamp = ros_clock_.now();
    static std::deque<BallPositionStamped> ball_records;
    ball_records.push_front(record);

    auto latest_time = ball_records.front().stamp;
    auto latest_position = ball_records.front().position;
    // 一定時間以上前の履歴を削除
    ball_records.erase(
      std::remove_if(
        ball_records.begin(), ball_records.end(),
        [&](auto & record) {
          return (latest_time - record.stamp) > config_.ball_idle.threshold_duration * 2;
        }),
      ball_records.end());

    // ボール履歴（新しいほど，indexが若い）のチェックして，ボールがストップしているかを確認
    for (auto record : ball_records) {
      if (
        (latest_position - record.position).norm() <
        config_.ball_idle.move_distance_threshold_meter) {
        if ((latest_time - record.stamp) < config_.ball_idle.threshold_duration) {
          return false;
        }
      }
    }
    return true;
  }

  std::optional<RobotCollisionInfo> getRobotCollisionInfo()
  {
    // TODO
    return std::nullopt;
  }

  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub_;

  WorldModelWrapper::UniquePtr world_model_;

  rclcpp::Publisher<crane_msgs::msg::GameAnalysis>::SharedPtr game_analysis_pub_;

  GameAnalyzerConfig config_;

  rclcpp::Clock ros_clock_;
};
}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_
