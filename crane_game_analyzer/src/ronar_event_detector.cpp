// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/ronar_event_detector.hpp"

#include <cmath>

namespace crane
{
RonarEventDetector::RonarEventDetector(rclcpp::Clock::SharedPtr clock)
: clock_(clock),
  possession_hysteresis_(0.15, 0.03),
  shot_start_time_(clock->now())
{
}

auto RonarEventDetector::detect(const WorldModelWrapper & world_model)
  -> std::vector<crane_msgs::msg::RonarEvent>
{
  std::vector<crane_msgs::msg::RonarEvent> events;

  // 各種イベント検出
  if (auto event = detectPossessionChange(world_model)) {
    events.push_back(*event);
  }

  if (auto event = detectShot(world_model)) {
    events.push_back(*event);
  }

  if (auto event = detectSetPlay(world_model)) {
    events.push_back(*event);
  }

  // 状態の更新（次フレーム用）
  last_play_situation_ = static_cast<uint8_t>(world_model.getMsg().play_situation.command.value);
  last_ball_pos_ = world_model.ball().pos;
  last_ball_vel_ = world_model.ball().vel;

  return events;
}

auto RonarEventDetector::detectPossessionChange(const WorldModelWrapper & wm)
  -> std::optional<crane_msgs::msg::RonarEvent>
{
  auto nearest = findNearestRobotToBall(wm);
  if (!nearest) {
    return std::nullopt;
  }

  auto [robot_id, distance] = *nearest;

  // ヒステリシスを適用
  possession_hysteresis_.update(distance);

  // ボールに十分近いロボットがいる場合のみ所持判定
  if (!possession_hysteresis_.is_high && distance < config_.possession_distance_threshold) {
    // 所持者が変わった場合のみイベント発火
    if (!current_possession_ || current_possession_->id != robot_id.id ||
        current_possession_->is_ours != robot_id.is_ours) {
      auto old_possession = current_possession_;
      current_possession_ = robot_id;

      // 履歴に追加
      PossessionRecord record{robot_id, clock_->now(), wm.ball().pos};
      possession_history_.push_front(record);
      if (possession_history_.size() > MAX_POSSESSION_HISTORY) {
        possession_history_.pop_back();
      }

      // イベント作成
      return createEvent(
        crane_msgs::msg::RonarEvent::EVENT_POSSESSION_CHANGE, wm.ball().pos, wm.ball().vel.norm(),
        0.9f, robot_id, old_possession);
    }
  }

  return std::nullopt;
}

auto RonarEventDetector::detectShot(const WorldModelWrapper & wm)
  -> std::optional<crane_msgs::msg::RonarEvent>
{
  double ball_speed = wm.ball().vel.norm();

  // シュート開始検出
  if (!shot_in_progress_ && ball_speed > 3.0 && isBallMovingTowardsGoal(wm, true)) {
    shot_in_progress_ = true;
    shot_start_time_ = clock_->now();
    if (current_possession_) {
      shot_kicker_ = *current_possession_;
    }

    uint8_t event_type = (ball_speed > config_.shot_velocity_threshold)
                           ? crane_msgs::msg::RonarEvent::EVENT_FAST_SHOT
                           : crane_msgs::msg::RonarEvent::EVENT_SHOT;

    return createEvent(event_type, wm.ball().pos, ball_speed, 0.85f, current_possession_);
  }

  // シュート終了判定
  if (shot_in_progress_) {
    // ボールが止まったか、方向が変わったらシュート終了
    if (ball_speed < 1.0 || !isBallMovingTowardsGoal(wm, true)) {
      shot_in_progress_ = false;
    }
  }

  return std::nullopt;
}

auto RonarEventDetector::detectPass(const WorldModelWrapper & wm)
  -> std::optional<crane_msgs::msg::RonarEvent>
{
  // TODO: KickEventDetectorと統合してパス検出を実装
  // キックイベント発生 + 受け手検出でPASSイベントを生成
  return std::nullopt;
}

auto RonarEventDetector::detectSetPlay(const WorldModelWrapper & wm)
  -> std::optional<crane_msgs::msg::RonarEvent>
{
  uint8_t current_situation = static_cast<uint8_t>(wm.getMsg().play_situation.command.value);

  // プレー状況が変わった場合
  if (current_situation != last_play_situation_) {
    // セットプレー系の状況変化を検出
    // KICKOFF, PENALTY, DIRECT_FREE, INDIRECT_FREE, BALL_PLACEMENT
    bool is_set_play =
      (current_situation >= 11 && current_situation <= 19) ||   // OUR系
      (current_situation >= 21 && current_situation <= 29);     // THEIR系

    if (is_set_play) {
      return createEvent(
        crane_msgs::msg::RonarEvent::EVENT_SET_PLAY, wm.ball().pos, 0.0f, 1.0f);
    }
  }

  return std::nullopt;
}

auto RonarEventDetector::findNearestRobotToBall(const WorldModelWrapper & wm) const
  -> std::optional<std::pair<RobotIdentifier, double>>
{
  double min_distance = std::numeric_limits<double>::max();
  std::optional<RobotIdentifier> nearest_robot;

  // 自チームロボットをチェック
  for (const auto & robot : wm.ours().getAvailableRobots()) {
    double dist = (robot->pose.pos - wm.ball().pos).norm();
    if (dist < min_distance) {
      min_distance = dist;
      nearest_robot = RobotIdentifier{.is_ours = true, .id = robot->id};
    }
  }

  // 相手チームロボットをチェック
  for (const auto & robot : wm.theirs().getAvailableRobots()) {
    double dist = (robot->pose.pos - wm.ball().pos).norm();
    if (dist < min_distance) {
      min_distance = dist;
      nearest_robot = RobotIdentifier{.is_ours = false, .id = robot->id};
    }
  }

  if (nearest_robot) {
    return std::make_pair(*nearest_robot, min_distance);
  }
  return std::nullopt;
}

auto RonarEventDetector::isBallMovingTowardsGoal(const WorldModelWrapper & wm, bool their_goal)
  const -> bool
{
  Point ball_vel = wm.ball().vel;
  if (ball_vel.norm() < 1.0) {
    return false;
  }

  // ゴール中心位置
  Point goal_center = their_goal ? wm.getTheirGoalCenter() : wm.getOurGoalCenter();
  Point ball_pos = wm.ball().pos;

  // ボール→ゴール方向
  Point to_goal = goal_center - ball_pos;

  // 速度ベクトルとゴール方向の角度
  double dot = ball_vel.normalized().dot(to_goal.normalized());

  // cos(30°) ≈ 0.866
  return dot > 0.866;
}

auto RonarEventDetector::createEvent(
  uint8_t type, const Point & position, float ball_speed, float confidence,
  const std::optional<RobotIdentifier> & primary,
  const std::optional<RobotIdentifier> & secondary) const -> crane_msgs::msg::RonarEvent
{
  crane_msgs::msg::RonarEvent event;
  event.header.stamp = clock_->now();
  event.event_type = type;
  event.position.x = position.x();
  event.position.y = position.y();
  event.position.z = 0.0;
  event.ball_speed = ball_speed;
  event.confidence = confidence;

  if (primary) {
    event.has_primary_robot = true;
    event.primary_robot_id = primary->id;
    event.primary_robot_is_ours = primary->is_ours;
  } else {
    event.has_primary_robot = false;
  }

  if (secondary) {
    event.has_secondary_robot = true;
    event.secondary_robot_id = secondary->id;
    event.secondary_robot_is_ours = secondary->is_ours;
  } else {
    event.has_secondary_robot = false;
  }

  event.metadata_json = "{}";

  return event;
}

}  // namespace crane
