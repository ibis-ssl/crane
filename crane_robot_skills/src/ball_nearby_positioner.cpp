// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_msgs/msg/play_situation.hpp>
#include <crane_robot_skills/ball_nearby_positioner.hpp>

namespace crane::skills
{
auto BallNearByPositioner::update() -> Status
{
  double distance_from_target = 0.5;

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
        if (
          auto theirs = world_model()->theirs().robotsWhere().available().get();
          theirs.size() >= 2) {
          auto nearest_robot = world_model()->getNearestRobotWithDistanceFromPoint(target, theirs);
          std::erase_if(theirs, [&](const auto & r) { return r->id == nearest_robot->robot->id; });
          auto second_nearest_robot =
            world_model()->getNearestRobotWithDistanceFromPoint(target, theirs);
          Vector2 dir = second_nearest_robot->robot->pose.pos - target;
          if (dir.squaredNorm() > 1e-6) {
            return dir.normalized();
          }
        }
        // 敵ロボットが2台未満、または方向ベクトルが無効な場合：ゴール方向をフォールバック
        return (world_model()->getOurGoalCenter() - target).normalized();
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

  auto avoidEnemyPenaltyArea = [&](Point & point) {
    const auto cmd = world_model()->getMsg().play_situation.command.value;
    using PS = crane_msgs::msg::PlaySituation;
    // INPLAY/HALT/HALF_TIME/POST_GAME以外（セットプレイ中）は拡大マージンを使用
    // rvo2_planner の needsExpandedPenaltyAreaOffset() と同等の判定
    const bool is_setplay =
      (cmd != PS::INPLAY && cmd != PS::HALT && cmd != PS::HALF_TIME && cmd != PS::POST_GAME);
    const double penalty_offset = is_setplay ? 0.2 : 0.15;

    if (world_model()->point_checker.isEnemyPenaltyArea(point, penalty_offset)) {
      const auto their_penalty_area = world_model()->getTheirPenaltyArea();
      const Point penalty_center =
        (their_penalty_area.min_corner() + their_penalty_area.max_corner()) * 0.5;
      Vector2 escape_direction = point - penalty_center;
      if (escape_direction.squaredNorm() < 1e-6) {
        escape_direction = Vector2(-1.0, 0.0);
      } else {
        escape_direction = escape_direction.normalized();
      }

      constexpr double step = 0.05;
      for (int i = 0;
           i < 100 && world_model()->point_checker.isEnemyPenaltyArea(point, penalty_offset); ++i) {
        point += escape_direction * step;
      }

      if (world_model()->point_checker.isEnemyPenaltyArea(point, penalty_offset)) {
        point.x() = their_penalty_area.min_corner().x() - penalty_offset;
      }
    }
  };

  avoidEnemyPenaltyArea(target_position);

  command->setTargetPosition(target_position).lookAtBall();
  if (
    world_model()->getMsg().play_situation.command.value ==
    crane_msgs::msg::PlaySituation::THEIR_BALL_PLACEMENT) {
    command->enablePlacementAvoidance();
  }
  return Status::RUNNING;
}
}  // namespace crane::skills
