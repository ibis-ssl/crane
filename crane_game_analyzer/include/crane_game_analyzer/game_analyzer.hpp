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

namespace crane
{
struct BallIdleConfig
{
  rclcpp::Duration threshold_duration = rclcpp::Duration(5, 0);
  double move_distance_threshold_meter = 0.05;
};

struct BallTouchConfig
{
  double touch_threshold_meter_ = 0.05;
};
struct GameAnalyzerConfig
{
  BallIdleConfig ball_idle;
  BallTouchConfig ball_touch;
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
  std::optional<BallTouchInfo> getBallTouchInfo()
  {
    BallTouchInfo info;
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

    double ours_distance = (nearest_ours->pose.pos - ball_pos).norm();
    double theirs_distance = (nearest_theirs->pose.pos - ball_pos).norm();

    if (ours_distance < theirs_distance) {
      info.robot_id.is_ours = true;
      info.robot_id.robot_id = nearest_ours->id;
      info.distance = ours_distance;
    } else {
      info.robot_id.is_ours = false;
      info.robot_id.robot_id = nearest_theirs->id;
      info.distance = theirs_distance;
    }
    if (info.distance > config_.ball_touch.touch_threshold_meter_) {
      return std::nullopt;
    }
    return std::make_optional(info);
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
    //delete records over threshold
    ball_records.erase(
      std::remove_if(
        ball_records.begin(), ball_records.end(),
        [&](auto & record) {
          return (latest_time - record.stamp) > config_.ball_idle.threshold_duration * 2;
        }),
      ball_records.end());

    // earlier first , older next
    // check boll idling
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

  WorldModelWrapper::SharedPtr world_model_;

  GameAnalyzerConfig config_;
  rclcpp::Clock ros_clock_;
};
}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__GAME_ANALYZER_HPP_
