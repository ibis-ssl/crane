// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/receive.hpp>

namespace crane::skills
{
Status Receive::update()
{
  auto offset = [&]() -> Point {
    Point offset(0, 0);
    if (getParameter<bool>("enable_software_bumper")) {
      command->addStateFactor("Receive", "enable software bumper");
      // ボール到着まで残り<software_bumper_start_time>秒になったら、ボール速度方向に少し加速して衝撃を和らげる
      double ball_speed = world_model()->ball.vel.norm();
      if (
        robot()->getDistance(world_model()->ball.pos) <
        ball_speed * getParameter<double>("software_bumper_start_time")) {
        // ボールから逃げ切らないようにするため、速度の0.5倍に制限
        command->setMaxVelocity(ball_speed * 0.5);
        // ボール速度方向に速度の0.5倍だけオフセット（1m/sで近づいていたら0.5m）
        offset += world_model()->ball.vel.normalized() * (world_model()->ball.vel.norm() * 0.5);
      }
    }
    if (getParameter<bool>("enable_active_receive")) {
      command->addStateFactor("Receive", "enable active receive");
      if (world_model()->ball.isMovingTowards(robot()->pose.pos, 2.0, 0.5)) {
        offset += (world_model()->ball.pos - robot()->pose.pos);
        double distance = (world_model()->ball.pos - robot()->pose.pos).norm();
        command->setMaxVelocity(distance);
      }
    }
    return offset;
  }();
  Point interception_point = getInterceptionPoint() + offset;

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
      Vector2 to_ball = world_model()->ball.pos - interception_point;
      Vector2 to_target = redirect_target - interception_point;
      // ボールとターゲットの角度の中間角を求める（暫定実装）
      return getIntermediateAngle(getAngle(to_ball), getAngle(to_target));
    }();
    command->dribble(0.0)
      .kickStraight(getParameter<double>("redirect_kick_power"))
      .setTargetTheta(target_angle);
  } else {
    command->lookAtBallFrom(interception_point).kickStraight(0.);
  }
  command->setDribblerTargetPosition(interception_point).disableBallAvoidance();

  return Status::RUNNING;
}

Point Receive::getInterceptionPoint() const
{
  Segment ball_line(
    world_model()->ball.pos,
    (world_model()->ball.pos + world_model()->ball.vel.normalized() * 10.0));
  Point closest_point = getClosestPointAndDistance(robot()->pose.pos, ball_line).closest_point;
  if (robot()->getDistance(closest_point) < 0.1) {
    return closest_point;
  }

  std::string policy = getParameter<std::string>("policy");
  auto acc = getParameter<double>("robot_acc_for_prediction");
  auto max_vel = getParameter<double>("robot_max_vel_for_prediction");
  command->addStateFactor("Receive::policy", policy);
  if (policy.ends_with("slack")) {
    auto slack_times = world_model()->getSlackInterceptPointAndSlackTimeArray(
      {robot()}, 3.0, 0.1, 0.2, acc, max_vel, world_model()->getMsg().game_analysis.ball_horizon);

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

    // マイナスのスラックタイムは削除
    slack_times.erase(
      std::remove_if(
        slack_times.begin(), slack_times.end(),
        [](const auto & slack) { return slack.slack_time < 0; }),
      slack_times.end());

    if (slack_times.empty()) {
      return getClosestPointAndDistance(robot()->pose.pos, ball_line).closest_point;
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
    if (policy == "max_slack" && max_slack != slack_times.end()) {
      return max_slack->intercept_point;
    } else if (policy == "min_slack" && min_slack != slack_times.end()) {
      return min_slack->intercept_point;
    }
    return world_model()->ball.pos;
  } else if (policy == "closest") {
    visualizer->line()
      .start(ball_line.first)
      .end(ball_line.second)
      .stroke("blue")
      .strokeWidth(10)
      .build();
    return getClosestPointAndDistance(robot()->pose.pos, ball_line).closest_point;
  } else {
    throw std::runtime_error("Invalid policy for Receive::getInterceptionPoint: " + policy);
  }
}
}  // namespace crane::skills
