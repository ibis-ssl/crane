// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_robot_skills/receive.hpp>

namespace crane::skills
{
Receive::Receive(RobotCommandWrapperBase::SharedPtr & base) : SkillBase("Receive", base)
{
  setParameter("dribble_power", 0.3);
  setParameter("enable_software_bumper", true);
  setParameter("software_bumper_start_time", 0.5);
  // min_slack, max_slack, closest
  setParameter("policy", std::string("closest"));
  setParameter("enable_active_receive", true);
  setParameter("enable_redirect", false);
  setParameter("redirect_target", Point(0, 0));
  setParameter("redirect_kick_power", 0.3);
}

Status Receive::update(const ConsaiVisualizerWrapper::SharedPtr & visualizer)
{
  auto offset = [&]() -> Point {
    Point offset(0, 0);
    if (getParameter<bool>("enable_software_bumper")) {
      // ボール到着まで残り<software_bumper_start_time>秒になったら、ボール速度方向に少し加速して衝撃を和らげる
      double ball_speed = world_model()->ball.vel.norm();
      if (
        robot()->getDistance(world_model()->ball.pos) <
        ball_speed * getParameter<double>("software_bumper_start_time")) {
        // ボールから逃げ切らないようにするため、速度の0.5倍に制限
        command.setMaxVelocity(ball_speed * 0.5);
        // ボール速度方向に速度の0.5倍だけオフセット（1m/sで近づいていたら0.5m）
        offset += world_model()->ball.vel.normalized() * (world_model()->ball.vel.norm() * 0.5);
      }
    }
    if (getParameter<bool>("enable_active_receive")) {
      if (world_model()->ball.isMovingTowards(robot()->pose.pos, 2.0, 0.5)) {
        offset += (world_model()->ball.pos - robot()->pose.pos);
        double distance = (world_model()->ball.pos - robot()->pose.pos).norm();
        command.setMaxVelocity(distance);
      }
    }
    return offset;
  }();
  Point interception_point = getInterceptionPoint(visualizer) + offset;

  visualizer->addLine(interception_point, robot()->pose.pos, 1, "red", 1., "intercept");

  if (getParameter<bool>("enable_redirect")) {
    Point redirect_target = getParameter<Point>("redirect_target");
    auto target_angle = [&]() {
      Vector2 to_ball = world_model()->ball.pos - interception_point;
      Vector2 to_target = redirect_target - interception_point;
      // ボールとターゲットの角度の中間角を求める（暫定実装）
      return getIntermediateAngle(getAngle(to_ball), getAngle(to_target));
    }();
    command.dribble(0.0)
      .kickStraight(getParameter<double>("redirect_kick_power"))
      .setTargetTheta(target_angle);
  } else {
    command.lookAtBallFrom(interception_point);
  }
  command.setDribblerTargetPosition(interception_point).disableBallAvoidance();

  return Status::RUNNING;
}

Point Receive::getInterceptionPoint(const ConsaiVisualizerWrapper::SharedPtr & visualizer) const
{
  std::string policy = getParameter<std::string>("policy");
  if (policy.ends_with("slack")) {
    auto [min_slack, max_slack] = world_model()->getMinMaxSlackInterceptPoint(
      {robot()}, 3.0, 0.1, 0., 4, 4, world_model()->getBallDistanceHorizon());
    if (policy == "max_slack" && max_slack) {
      return max_slack.value();
    } else if (policy == "min_slack" && min_slack) {
      return min_slack.value();
    }
    return world_model()->ball.pos;
  } else if (policy == "closest") {
    Segment ball_line(
      world_model()->ball.pos,
      (world_model()->ball.pos + world_model()->ball.vel.normalized() * 10.0));
    visualizer->addLine(ball_line.first, ball_line.second, 1, "blue", 1., "ball_line");
      auto result = getClosestPointAndDistance(robot()->pose.pos, ball_line);
      return result.closest_point;
    } else {
      throw std::runtime_error("Invalid policy for Receive::getInterceptionPoint: " + policy);
    }
  }
}  // namespace crane::skills
