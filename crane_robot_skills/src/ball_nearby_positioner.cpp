// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/ball_nearby_positioner.hpp>

namespace crane::skills
{
auto BallNearByPositioner::update() -> Status
{
  auto situation = world_model()->getMsg().play_situation.command.value;
  double distance_from_target = [&]() {
    switch (situation) {
      case crane_msgs::msg::PlaySituation::THEIR_DIRECT_FREE:
        return 0.5;
      case crane_msgs::msg::PlaySituation::STOP:
        return 0.5;
      case crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT:
        return 0.5;
      default:
        return 0.0;
    }
  }();

  distance_from_target += getParameter<double>("margin_distance");
  double normalized_offset =
    (getParameter<int>("total_robot_number") - getParameter<int>("current_robot_index") - 1) / 2.;
  double offset = normalized_offset * getParameter<double>("robot_interval");

  Point target = [&]() {
    if (getParameter<bool>("alternative_target_mode")) {
      return getParameter<Point>("alternative_target");
    } else {
      return world_model()->ball().pos;
    }
  }();
  Point base_position =
    target +
    [&](std::string policy) {
      if (policy == "auto") {
        if (
          world_model()->getLargestOurGoalAngleRangeFromPoint(target).angle_width <
          5.0 * boost::math::constants::degree<double>()) {
          policy = "pass";
        } else {
          policy = "goal";
        }
      }

      if (policy == "goal") {
        return (world_model()->getOurGoalCenter() - target).normalized();
      } else if (policy == "pass") {
        // 2番目に近いロボット
        if (auto theirs = world_model()->theirs().getAvailableRobots(); theirs.size() >= 2) {
          auto nearest_robot = world_model()->getNearestRobotWithDistanceFromPoint(target, theirs);
          std::erase_if(theirs, [&](const auto & r) { return r->id == nearest_robot->robot->id; });
          auto second_nearest_robot =
            world_model()->getNearestRobotWithDistanceFromPoint(target, theirs);
          return (second_nearest_robot->robot->pose.pos - target).normalized();
        } else {
          // 敵ロボットが2台未満の場合：ゴール方向をフォールバック
          return (world_model()->getOurGoalCenter() - target).normalized();
        }
      } else {
        throw std::runtime_error(
          "[BallNearByPositioner] "
          "予期しないパラメータ「positioning_policy」が入力されています: " +
          policy);
      }
    }(getParameter<std::string>("positioning_policy")) *
      distance_from_target;

  Point target_position = [&](const std::string & policy) -> Point {
    Vector2 target_to_base = (base_position - target);
    if (policy == "arc") {
      double base_angle = getAngle(target_to_base);
      // r x theta = interval
      // theta = interval / r
      double angle_interval = getParameter<double>("robot_interval") / distance_from_target;
      return target +
             getNormVec(base_angle + normalized_offset * angle_interval) * distance_from_target;
    } else if (policy == "straight") {
      return target_to_base + getVerticalVec(target_to_base.normalized()) * offset;
    } else {
      throw std::runtime_error(
        "[BallNearByPositioner] 予期しないパラメータ「line_policy」が入力されています: " + policy);
    }
  }(getParameter<std::string>("line_policy"));

  command->setTargetPosition(target_position).lookAtBall();
  if (
    world_model()->getMsg().play_situation.command.value ==
    crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT) {
    command->enablePlacementAvoidance();
  }
  return Status::RUNNING;
}
}  // namespace crane::skills
