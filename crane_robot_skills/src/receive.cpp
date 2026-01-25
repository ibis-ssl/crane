// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <crane_robot_skills/receive.hpp>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

namespace crane::skills
{
Status Receive::update()
{
  auto offset = [&]() -> Point {
    Point offset(0, 0);
    if (getParameter<bool>("enable_software_bumper")) {
      command->addPlanningFactor("Receive", "enable software bumper");
      // ボール到着まで残り<software_bumper_start_time>秒になったら、ボール速度方向に少し加速して衝撃を和らげる
      double ball_speed = world_model()->ball().vel.norm();
      if (
        robot()->getDistance(world_model()->ball().pos) <
        ball_speed * getParameter<double>("software_bumper_start_time")) {
        command->setMaxVelocity(
          "ボールから逃げ切らないようにするため、速度の0.5倍に制限", ball_speed * 0.5);
        // ボール速度方向に速度の0.5倍だけオフセット（1m/sで近づいていたら0.5m）
        offset += world_model()->ball().vel.normalized() * (world_model()->ball().vel.norm() * 0.5);
      }
    }
    if (getParameter<bool>("enable_active_receive")) {
      command->addPlanningFactor("Receive", "enable active receive");
      if (world_model()->ball().isMovingTowards(robot()->pose.pos, 2.0, 0.5)) {
        offset += (world_model()->ball().pos - robot()->pose.pos);
        double distance = (world_model()->ball().pos - robot()->pose.pos).norm();
        command->setMaxVelocity("Receive::active_receive", distance);
      }
    }
    return offset;
  }();

  // Base interception point before offset (also draws candidate visuals inside)
  Point base_interception_point = getInterceptionPoint();
  Point interception_point = base_interception_point + offset;

  // Minimal HUD near robot: policy only
  visualizer->text()
    .position(robot()->pose.pos + Vector2(0.0, 0.35))
    .text(std::string("Receive[") + getParameter<std::string>("policy") + "]")
    .fontSize(60)
    .fill("white")
    .textAnchor("middle")
    .build();

  // Offset arrow (from base to final) for clarity
  if (getParameter<bool>("viz_offset_arrow")) {
    Vector2 off_vec = interception_point - base_interception_point;
    if (off_vec.squaredNorm() > 1e-6) {
      visualizer->arrow(base_interception_point, off_vec, off_vec.norm(), "cyan", 8);
    }
  }

  if (getParameter<bool>("enable_redirect")) {
    command->addPlanningFactor("Receive", "enable redirect");
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

    // Redirect preview (incoming and outgoing directions)
    if (getParameter<bool>("viz_redirect_preview")) {
      Vector2 in_vec = interception_point - world_model()->ball().pos;
      if (in_vec.norm() < 1e-6 && world_model()->ball().vel.norm() > 1e-6) {
        in_vec =
          interception_point - (interception_point - world_model()->ball().vel.normalized() * 0.3);
      }
      Vector2 out_vec = redirect_target - interception_point;
      if (in_vec.norm() > 1e-6) {
        visualizer->arrow(
          interception_point - in_vec.normalized() * 0.35, in_vec.normalized(), 0.35, "red", 10);
      }
      if (out_vec.norm() > 1e-6) {
        visualizer->arrow(interception_point, out_vec.normalized(), 0.6, "lime", 10);
      }
    }
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

  // Optionally draw ball trajectory thin and semi-transparent for both policies
  if (getParameter<bool>("viz_ball_traj")) {
    visualizer->drawLine(ball_line.first, ball_line.second, "#00aaff", 6, 0.5);
  }

  // closest_pointのNaN値チェック
  if (!isValidPoint(closest_point)) {
    RCLCPP_WARN(
      rclcpp::get_logger("Receive"), "closest_point is NaN, falling back to robot position");
    command->addPlanningFactor(
      "Receive", "closest_pointがNaN値のため、ロボット位置をフォールバック使用");
    closest_point = robot()->pose.pos;
  }

  if (robot()->getDistance(closest_point) < 0.1) {
    command->addPlanningFactor(
      "Receive", "ロボットがclosest_pointに十分近いため、closest policyを強制適用");
    return closest_point;
  }

  std::string policy = getParameter<std::string>("policy");
  command->addPlanningFactor("Receive::policy", policy);

  if (policy.ends_with("slack")) {
    auto slack_times = world_model()->getSlackInterceptPointAndSlackTimeArray({robot()});
    // Slack color mapper: [-0.5, 0, +0.5] -> red, yellow, green
    auto slackColor = [](double s) -> std::string {
      double v = std::clamp(s, -0.5, 0.5);
      // map to [0,1]
      double t = (v + 0.5) / 1.0;
      // simple red->yellow->green gradient
      int r = 0;
      int g = 0;
      if (t < 0.5) {
        // red (255,0,0) to yellow (255,255,0)
        double k = t / 0.5;
        r = 255;
        g = static_cast<int>(255 * k);
      } else {
        // yellow (255,255,0) to green (0,200,0)
        double k = (t - 0.5) / 0.5;
        r = static_cast<int>(255 * (1.0 - k));
        g = static_cast<int>(255 - 55 * k);  // 255 -> ~200
      }
      std::ostringstream oss;
      oss << "rgb(" << r << "," << g << ",0)";
      return oss.str();
    };

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
      RCLCPP_WARN(
        rclcpp::get_logger("Receive"), "slack_times is empty, falling back to closest point");
      command->addPlanningFactor("Receive", message);
      Point fallback_point = getClosestPointAndDistance(robot()->pose.pos, ball_line).closest_point;
      if (!isValidPoint(fallback_point)) {
        // ball_lineもNaN値の場合、ロボット現在位置をフォールバック
        std::string message = "WARN: [Receive] fallback_pointもNaN値のため、ロボット位置を使用";
        RCLCPP_WARN(
          rclcpp::get_logger("Receive"),
          "fallback_point is also NaN, falling back to robot position");
        command->addPlanningFactor("Receive", message);
        return robot()->pose.pos;
      }
      return fallback_point;
    }

    auto [min_slack, max_slack] = std::minmax_element(
      slack_times.begin(), slack_times.end(),
      [](const auto & a, const auto & b) { return a.slack_time < b.slack_time; });
    // Draw candidate points colored by slack
    if (getParameter<bool>("viz_candidates")) {
      for (const auto & s : slack_times) {
        visualizer->drawStyledCircle(
          s.intercept_point, 0.06, slackColor(s.slack_time), 0.6, "black", 0.3, 4);
      }
    }

    Point selected_point;
    double selected_slack = 0.0;
    if (policy == "max_slack" && max_slack != slack_times.end()) {
      selected_point = max_slack->intercept_point;
      selected_slack = max_slack->slack_time;
    } else if (policy == "min_slack" && min_slack != slack_times.end()) {
      selected_point = min_slack->intercept_point;
      selected_slack = min_slack->slack_time;
    } else {
      selected_point = world_model()->ball().pos;
    }

    // 選択されたポイントのNaN値チェック
    if (!isValidPoint(selected_point)) {
      std::string message = "WARN: [Receive] selected_pointがNaN値のため、ロボット位置を使用";
      RCLCPP_WARN(
        rclcpp::get_logger("Receive"), "selected_point is NaN, falling back to robot position");
      command->addPlanningFactor("Receive", message);
      return robot()->pose.pos;
    }

    // Emphasize selected point with double circle and small label
    if (getParameter<bool>("viz_candidates")) {
      visualizer->doubleCircle(selected_point, 0.06, 0.09, "#ffffff", "#222", 0, 8);
      std::ostringstream ss;
      if (policy == "max_slack") ss << "max ";
      if (policy == "min_slack") ss << "min ";
      ss << std::fixed << std::setprecision(2) << selected_slack << "s";
      visualizer->drawCenteredLabel(selected_point + Vector2(0.0, -0.18), ss.str(), "white", 60);
    }
    return selected_point;

  } else if (policy == "closest") {
    // Highlight closest point slightly and annotate
    visualizer->labeledCircle(closest_point, 0.06, "closest", "#222", "white", 6, 50);
    return closest_point;
  } else {
    throw std::runtime_error("Invalid policy for Receive::getInterceptionPoint: " + policy);
  }
}
}  // namespace crane::skills
