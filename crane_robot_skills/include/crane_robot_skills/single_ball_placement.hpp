// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
#define CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_

#include <crane_geometry/vector2d_adapter.hpp>
#include <crane_robot_skills/skill_base.hpp>
#include <memory>

#include "receive.hpp"
#include "sleep.hpp"

namespace crane::skills
{
enum class SingleBallPlacementStates {
  ENTRY_POINT,
  RECEIVE_BALL,
  PULL_BACK_FROM_EDGE_PREPARE,
  PULL_BACK_FROM_EDGE_TOUCH,
  PULL_BACK_FROM_EDGE_PULL,
  PULL_BACK_FROM_EDGE_OVER_SLEEP,
  PULL_BACK_FROM_EDGE_OVER_LEAVE,
  GO_OVER_BALL,
  PASS_TO_TARGET,
  CONTACT_BALL,
  MOVE_TO_TARGET,
  PLACE_BALL,
  SLEEP,
  LEAVE_BALL,
};

class SingleBallPlacement : public SkillBaseWithState
{
private:
  static std::string getStateName(int s);

  std::shared_ptr<Receive> receive;

  std::shared_ptr<Sleep> sleep = nullptr;

  Status skill_status = Status::RUNNING;

  std::optional<Point> pull_back_target;

  double pull_back_angle;

  int contact_count_ = 0;

  mutable std::optional<rclcpp::Time> last_ball_sensor_active_time_;

  Point getPlacementTarget() const
  {
    Point p;
    p << getParameter<double>("placement_x"), getParameter<double>("placement_y");
    return p;
  }

  /// ボールがドリブラーから確実に離れたかを判定する。
  /// Vision/Tracker両方が検出かつ位置が一致かつデータが新鮮な場合のみ判定可能。
  /// ボールセンサーが最後に反応してから0.1秒以内は喪失判定をスキップする。
  bool isBallTrulyLostFromDribbler(double distance_threshold = 0.2) const
  {
    const auto & ball_info = world_model()->getMsg().ball_info;
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();
    constexpr double FRESHNESS_SEC = 0.1;

    // ボールセンサーの最終反応時刻を更新
    if (robot()->ball_sensor) {
      last_ball_sensor_active_time_ = now;
    }

    // ボールセンサーが最近反応している場合はドリブル中と判断し喪失判定をスキップ
    constexpr double BALL_SENSOR_ACTIVE_TIMEOUT_SEC = 0.1;
    if (
      last_ball_sensor_active_time_.has_value() &&
      now.get_clock_type() == last_ball_sensor_active_time_->get_clock_type() &&
      (now - *last_ball_sensor_active_time_).seconds() <= BALL_SENSOR_ACTIVE_TIMEOUT_SEC) {
      auto clock = rclcpp::Clock(RCL_ROS_TIME).make_shared();
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("SingleBallPlacement"), *clock, 500,
        "[isBallTrulyLostFromDribbler] result=0 reason=ball_sensor_active ball_sensor=%d",
        robot()->ball_sensor);
      return false;
    }
    rclcpp::Time vision_stamp(ball_info.vision.stamp, RCL_ROS_TIME);
    rclcpp::Time tracker_stamp(ball_info.tracker.stamp, RCL_ROS_TIME);

    double vision_age = (vision_stamp.nanoseconds() > 0) ? (now - vision_stamp).seconds() : -1.0;
    double tracker_age = (tracker_stamp.nanoseconds() > 0) ? (now - tracker_stamp).seconds() : -1.0;

    Point vision_pos(ball_info.vision.pos.x, ball_info.vision.pos.y);
    Point tracker_pos(ball_info.tracker.pos.x, ball_info.tracker.pos.y);
    double vision_tracker_diff = (vision_pos - tracker_pos).norm();
    double dist_from_dribbler = (tracker_pos - robot()->kicker_center()).norm();

    bool vision_fresh = vision_age >= 0.0 && vision_age <= FRESHNESS_SEC;
    bool tracker_fresh = tracker_age >= 0.0 && tracker_age <= FRESHNESS_SEC;

    bool result = false;
    const char * reason = "unknown";
    if (!vision_fresh) {
      reason = "vision_not_fresh";
    } else if (!tracker_fresh) {
      reason = "tracker_not_fresh";
    } else if (vision_tracker_diff > 0.05) {
      reason = "vision_tracker_mismatch";
    } else {
      result = dist_from_dribbler > distance_threshold;
      reason = result ? "lost" : "still_close";
    }

    auto clock = rclcpp::Clock(RCL_ROS_TIME).make_shared();
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("SingleBallPlacement"), *clock, 500,
      "[isBallTrulyLostFromDribbler] result=%d reason=%s "
      "v_det=%d t_det=%d v_age=%.3fs t_age=%.3fs "
      "v_pos=(%.3f,%.3f) t_pos=(%.3f,%.3f) diff=%.3fm "
      "dist=%.3fm threshold=%.3fm",
      result, reason, ball_info.vision_detected, ball_info.tracker_detected, vision_age,
      tracker_age, vision_pos.x(), vision_pos.y(), tracker_pos.x(), tracker_pos.y(),
      vision_tracker_diff, dist_from_dribbler, distance_threshold);

    return result;
  }

public:
  template <typename... Args>
  explicit SingleBallPlacement(Args &&... args)
  : SkillBaseWithState(
      static_cast<int>(SingleBallPlacementStates::ENTRY_POINT), &SingleBallPlacement::getStateName,
      "SingleBallPlacement", std::forward<Args>(args)...)
  {
    initialize();
  }

  void initialize();

  SingleBallPlacementStates getCurrentState() const
  {
    return static_cast<SingleBallPlacementStates>(SkillBaseWithState::getCurrentState());
  }
};
}  // namespace crane::skills
#endif  // CRANE_ROBOT_SKILLS__SINGLE_BALL_PLACEMENT_HPP_
