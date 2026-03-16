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
  // ボール方向が90度以上変わったら前フレームインターセプト位置をリセット
  // (Sumatra BallInterceptor参考: 大きな方向変化は新規インターセプト位置を採用)
  if (prev_interception_point_) {
    constexpr double BALL_SPEED_RESET_THRESHOLD = 0.5;  // リセット判定の最低ボール速度 [m/s]
    constexpr double PREV_DIST_RESET_THRESHOLD = 0.1;   // リセット判定の最低距離 [m]
    auto ball_vel = world_model()->ball().vel;
    double ball_speed = ball_vel.norm();
    Vector2 to_prev = prev_interception_point_.value() - world_model()->ball().pos;
    double prev_dist = to_prev.norm();
    if (ball_speed > BALL_SPEED_RESET_THRESHOLD && prev_dist > PREV_DIST_RESET_THRESHOLD) {
      double cos_angle = (ball_vel / ball_speed).dot(to_prev / prev_dist);
      if (cos_angle < 0.0) {  // 90度以上の方向変化
        prev_interception_point_ = std::nullopt;
      }
    }
  }

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
  Point base_interception_point = getInterceptionPointWithEnemyAvoidance();
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

  if (
    policy.ends_with("slack") &&
    world_model()->ball().vel.norm() > world_model()->getSlackConfig().velocity_epsilon) {
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
      RCLCPP_WARN(
        rclcpp::get_logger("Receive"), "slack_times is empty, falling back to ball stop position");
      // ボール停止予測位置へ先回りする（減速モデル対応フォールバック）
      Point stop_pos =
        world_model()->ball().getPredictedPosition(world_model()->ball().getStopTime());
      if (isValidPoint(stop_pos) && world_model()->point_checker.isFieldInside(stop_pos)) {
        command->addPlanningFactor(
          "Receive", "WARN: [Receive] slack_timesが空のため、ボール停止予測位置にフォールバック");
        return stop_pos;
      }
      // フィールド外またはNaN値の場合は従来の closest point フォールバック
      command->addPlanningFactor(
        "Receive", "WARN: [Receive] 停止予測位置が無効のため、closest pointにフォールバック");
      Point fallback_point = getClosestPointAndDistance(robot()->pose.pos, ball_line).closest_point;
      if (!isValidPoint(fallback_point)) {
        command->addPlanningFactor(
          "Receive", "WARN: [Receive] fallback_pointもNaN値のため、ロボット位置を使用");
        RCLCPP_WARN(
          rclcpp::get_logger("Receive"),
          "fallback_point is also NaN, falling back to robot position");
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

    // 前フレーム位置の安定化ボーナス（Sumatra BallInterceptor参考）
    if (prev_interception_point_) {
      double dist_to_prev = (selected_point - prev_interception_point_.value()).norm();
      constexpr double STABILITY_DISTANCE_THRESHOLD = 0.5;  // 0.5m以内なら前回位置を優先
      if (dist_to_prev < STABILITY_DISTANCE_THRESHOLD) {
        selected_point = prev_interception_point_.value();
      }
    }
    prev_interception_point_ = selected_point;

    // Emphasize selected point with double circle and small label
    if (getParameter<bool>("viz_candidates")) {
      visualizer->doubleCircle(selected_point, 0.06, 0.09, "#ffffff", "#222", 8);
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

bool Receive::isEnemyBlockingPassLine(const Point & interception_point) const
{
  Segment pass_line{world_model()->ball().pos, interception_point};
  auto enemy_robots = world_model()->theirs().robotsWhere().available().get();

  if (
    auto nearest_enemy =
      world_model()->getNearestRobotWithDistanceFromSegment(pass_line, enemy_robots);
    nearest_enemy.has_value()) {
    constexpr double BLOCKING_THRESHOLD = 0.3;  // 敵がパスラインから0.3m以内
    return nearest_enemy->distance < BLOCKING_THRESHOLD;
  }
  return false;
}

Point Receive::getInterceptionPointWithEnemyAvoidance() const
{
  Point base_point = getInterceptionPoint();

  if (!isEnemyBlockingPassLine(base_point)) {
    return base_point;  // 敵がいなければそのまま
  }

  // 敵が割り込んでいる場合、Slack Timeベースで代替位置を探す
  auto slack_times = world_model()->getSlackInterceptPointAndSlackTimeArray({robot()});

  // 有効なNaN値チェックと敵ブロックチェック
  auto isValidPoint = [](const Point & p) -> bool {
    return std::isfinite(p.x()) && std::isfinite(p.y());
  };

  std::vector<WorldModelWrapper::SlackTimeResult> valid_points;
  for (const auto & slack : slack_times) {
    if (
      slack.slack_time > 0 && isValidPoint(slack.intercept_point) &&
      !isEnemyBlockingPassLine(slack.intercept_point)) {
      valid_points.push_back(slack);
    }
  }

  if (valid_points.empty()) {
    command->addPlanningFactor("Receive", "敵割り込み: 有効な代替位置なし");
    return base_point;  // 代替がなければ元の位置を維持
  }

  // 最もSlack Timeが大きい位置を選択
  auto best = std::max_element(
    valid_points.begin(), valid_points.end(),
    [](const auto & a, const auto & b) { return a.slack_time < b.slack_time; });

  command->addPlanningFactor("Receive", "敵割り込み: 代替位置に移動");

  // 可視化: 敵割り込み警告マーカー
  if (getParameter<bool>("viz_enemy_block")) {
    visualizer->drawCircle(base_point, 0.08, "red", 0.7, 8);
    visualizer->drawCircle(best->intercept_point, 0.08, "lime", 0.7, 8);
  }

  return best->intercept_point;
}
}  // namespace crane::skills
