// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/receive.hpp>
#include <iostream>
#include <ostream>
#include <string>

namespace crane::skills
{
Status Receive::update()
{
  auto offset = [&]() -> Point {
    Point offset(0, 0);
    if (getParameter<bool>("enable_software_bumper")) {
      command->addStateFactor("Receive", "enable software bumper");
      // ボール到着まで残り<software_bumper_start_time>秒になったら、ボール速度方向に少し加速して衝撃を和らげる
      double ball_speed = world_model()->ball().vel.norm();
      if (
        robot()->getDistance(world_model()->ball().pos) <
        ball_speed * getParameter<double>("software_bumper_start_time")) {
        // ボールから逃げ切らないようにするため、速度の0.5倍に制限
        command->setMaxVelocity(ball_speed * 0.5);
        // ボール速度方向に速度の0.5倍だけオフセット（1m/sで近づいていたら0.5m）
        offset += world_model()->ball().vel.normalized() * (world_model()->ball().vel.norm() * 0.5);
      }
    }
    if (getParameter<bool>("enable_active_receive")) {
      command->addStateFactor("Receive", "enable active receive");
      if (world_model()->ball().isMovingTowards(robot()->pose.pos, 2.0, 0.5)) {
        offset += (world_model()->ball().pos - robot()->pose.pos);
        double distance = (world_model()->ball().pos - robot()->pose.pos).norm();
        command->setMaxVelocity(distance);
      }
    }
    return offset;
  }();

  Point interception_point = getInterceptionPoint() + offset;

  command->addStateFactor(
    "Receive", "offset: " + std::to_string(offset.x()) + "," + std::to_string(offset.y()));
  command->addStateFactor(
    "Receive", "interception_point: " + std::to_string(interception_point.x()) + "," +
                 std::to_string(interception_point.y()));

  visualizer->line()
    .start(interception_point)
    .end(robot()->pose.pos)
    .stroke("red")
    .strokeWidth(10)
    .build();

  if (getParameter<bool>("enable_redirect")) {
    command->addStateFactor("Receive", "enable redirect");
    Point redirect_target = getParameter<Point>("redirect_target");
    auto target_angle = [&]() {
      Vector2 to_ball = world_model()->ball().pos - interception_point;
      Vector2 to_target = redirect_target - interception_point;
      // ボールとターゲットの角度の中間角を求める（暫定実装）
      return getIntermediateAngle(getAngle(to_ball), getAngle(to_target));
    }();
    command->dribble(0.0)
      .kickStraight(getParameter<double>("redirect_kick_power"))
      .setTargetTheta(target_angle);
  } else {
    command->lookAtBall().kickStraight(0.);
  }
  command->setDribblerTargetPosition(interception_point).disableBallAvoidance();

  return Status::RUNNING;
}

Point Receive::getInterceptionPoint() const
{
  // NaN値チェックのヘルパー関数
  auto isValidPoint = [](const Point & p) -> bool {
    return std::isfinite(p.x()) && std::isfinite(p.y());
  };

  Segment ball_line = world_model()->ball().getTrajectorySegmentByDistance(10.0);
  Point closest_point =
    world_model()->ball().getClosestPointToTrajectory(robot()->pose.pos, 10.0).closest_point;

  // closest_pointのNaN値チェック
  if (!isValidPoint(closest_point)) {
    std::cout << "WARN: [Receive] closest_pointがNaN値のため、ロボット位置をフォールバック使用"
              << std::endl;
    command->addStateFactor(
      "Receive", "closest_pointがNaN値のため、ロボット位置をフォールバック使用");
    closest_point = robot()->pose.pos;
  }

  if (robot()->getDistance(closest_point) < 0.1) {
    command->addStateFactor(
      "Receive", "ロボットがclosest_pointに十分近いため、closest policyを強制適用");
    return closest_point;
  }

  std::string policy = getParameter<std::string>("policy");
  auto acc = getParameter<double>("robot_acc_for_prediction");
  auto max_vel = getParameter<double>("robot_max_vel_for_prediction");
  command->addStateFactor("Receive::policy", policy);

  if (policy.ends_with("slack")) {
    auto slack_times = world_model()->getSlackInterceptPointAndSlackTimeArray(
      {robot()}, 3.0, 0.1, 0.5, acc, max_vel, world_model()->getMsg().game_analysis.ball_horizon);

    for (auto slack : slack_times) {
      visualizer->text()
        .position(slack.intercept_point)
        .text(std::to_string(slack.robot->id) + ": " + std::to_string(slack.slack_time))
        .fontSize(50)
        .fill([&]() -> std::string {
          if (slack.slack_time > 0) {
            return "black";
          } else {
            return "red";
          }
        }())
        .build();
    }

    // マイナスのスラックタイムとNaN値を含むエントリを削除
    slack_times.erase(
      std::remove_if(
        slack_times.begin(), slack_times.end(),
        [&](const auto & slack) {
          return slack.slack_time < 0 || !isValidPoint(slack.intercept_point);
        }),
      slack_times.end());

    if (slack_times.empty()) {
      std::string message = "WARN: [Receive] slack_timesが空のため、closest pointにフォールバック";
      std::cout << message << std::endl;
      command->addStateFactor("Receive", message);
      Point fallback_point = getClosestPointAndDistance(robot()->pose.pos, ball_line).closest_point;
      if (!isValidPoint(fallback_point)) {
        // ball_lineもNaN値の場合、ロボット現在位置をフォールバック
        std::string message = "WARN: [Receive] fallback_pointもNaN値のため、ロボット位置を使用";
        std::cout << message << std::endl;
        command->addStateFactor("Receive", message);
        return robot()->pose.pos;
      }
      return fallback_point;
    }

    auto [min_slack, max_slack] = std::minmax_element(
      slack_times.begin(), slack_times.end(),
      [](const auto & a, const auto & b) { return a.slack_time < b.slack_time; });

    if (max_slack != slack_times.end()) {
      std::string text = "max_slack: " + std::to_string(max_slack->slack_time);
      visualizer->text()
        .position(max_slack->intercept_point)
        .text(text)
        .fill("black")
        .fontSize(100)
        .build();
    }
    if (min_slack != slack_times.end()) {
      std::string text = "min_slack: " + std::to_string(min_slack->slack_time);
      visualizer->text()
        .position(min_slack->intercept_point)
        .text(text)
        .fill("black")
        .fontSize(100)
        .build();
    }

    Point selected_point;
    if (policy == "max_slack" && max_slack != slack_times.end()) {
      selected_point = max_slack->intercept_point;
    } else if (policy == "min_slack" && min_slack != slack_times.end()) {
      selected_point = min_slack->intercept_point;
    } else {
      selected_point = world_model()->ball().pos;
    }

    // 選択されたポイントのNaN値チェック
    if (!isValidPoint(selected_point)) {
      std::string message = "WARN: [Receive] selected_pointがNaN値のため、ロボット位置を使用";
      std::cout << message << std::endl;
      command->addStateFactor("Receive", message);
      return robot()->pose.pos;
    }
    return selected_point;

  } else if (policy == "closest") {
    visualizer->line()
      .start(ball_line.first)
      .end(ball_line.second)
      .stroke("blue")
      .strokeWidth(10)
      .build();
    return closest_point;
  } else {
    throw std::runtime_error("Invalid policy for Receive::getInterceptionPoint: " + policy);
  }
}
}  // namespace crane::skills
