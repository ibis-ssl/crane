// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_world_model_publisher/kick_event_detector.hpp"

namespace crane
{
auto KickEventDetector::update(
  const WorldModelWrapper & world_model,
  const VisualizerMessageBuilder::SharedPtr visualizer) -> void
{
  {
    Record record;
    record.position = world_model.ball().pos;
    record.velocity = world_model.ball().vel;
    records.emplace_back(record);
  }

  if (records.size() > QUEUE_SIZE) {
    records.pop_front();
  } else {
    return;
  }

  DetectedBots available_bots;
  available_bots.friends = world_model.ours().getAvailableRobotIds();
  available_bots.enemies = world_model.theirs().getAvailableRobotIds();

  auto detected_bots = filterByDistance(distance_threshold, available_bots, world_model);
  detected_bots = filterByVelocity(0.5, detected_bots, world_model);
  detected_bots = filterByBotAngle(0.5, detected_bots, world_model);
  detected_bots = filterByDistanceIncrease(detected_bots, world_model);
  // print detected bots

  std::optional<KickOrigin> kick_event_origin = std::nullopt;
  for (const auto & id : detected_bots.friends) {
    if (visualizer) {
        visualizer->circle()
                  .center(world_model.getOurRobot(id)->pose.pos)
                  .radius(0.5)
                  .stroke("blue")
                  .fill("blue", 0.3)
                  .strokeWidth(20)
                  .build();
      }
    kick_event_origin.emplace(ros_clock.now(), world_model.ball().pos, RobotIdentifier{true, id});
  }

  for (const auto & id : detected_bots.enemies) {
      if (visualizer) {
        visualizer->circle()
                  .center(world_model.getTheirRobot(id)->pose.pos)
                  .radius(0.5)
                  .stroke("blue")
                  .fill("blue", 0.3)
                  .strokeWidth(20)
                  .build();
      }
    kick_event_origin.emplace(
      ros_clock.now(), world_model.ball().pos, RobotIdentifier{false, id});
  }

  // 進行中キックの更新
  if (kick_event_origin.has_value()) {
    ongoing_kick_origin = kick_event_origin.value();
  } else {
    if (ongoing_kick_origin.has_value() && hasInterruptedOnGoingKick(world_model)) {
      // キック中断判定
      kick_history.emplace_back(ongoing_kick_origin.value(), world_model.ball().pos);
      ongoing_kick_origin = std::nullopt;
    }
  }

  // 進行中のキックを可視化
  if (ongoing_kick_origin.has_value()) {
    if (visualizer) {
      visualizer->line()
                .start(ongoing_kick_origin.value().position)
                .end(world_model.ball().pos)
                .stroke("red", 0.3)
                .strokeWidth(200)
                .build();
    }
  }
}

auto KickEventDetector::getOnGoingKick() -> std::optional<crane_msgs::msg::Kick>
{
  if (ongoing_kick_origin.has_value()) {
    auto kick = crane_msgs::msg::Kick();
    kick.kicker_id = ongoing_kick_origin->robot.id;
    kick.is_kicker_friend = ongoing_kick_origin->robot.is_ours;
    kick.origin_x = ongoing_kick_origin->position.x();
    kick.origin_y = ongoing_kick_origin->position.y();
    kick.direction = atan2(
      records.back().position.y() - ongoing_kick_origin->position.y(),
      records.back().position.x() - ongoing_kick_origin->position.x());
    return kick;
  } else {
    return std::nullopt;
  }
}

auto KickEventDetector::hasInterruptedOnGoingKick(
  const WorldModelWrapper & world_model) const -> bool
{
  const auto & latest = records.back();
  const auto & pre = records.at(records.size() - 2);
  double pre_vel = pre.velocity.norm();
  double vel_diff = (latest.velocity - pre.velocity).norm();

  double score = vel_diff / (pre_vel + 0.1) * 100;
  bool event_detected = score > 30;

  return world_model.ball().isStopped(0.5) or event_detected;
}

auto KickEventDetector::filterByDistance(
  double threshold, const DetectedBots & available_bots,
  const WorldModelWrapper & world_model) -> DetectedBots
{
  using ranges::views::filter;
  DetectedBots detected_bots;
  detected_bots.friends = available_bots.friends | filter([&](const auto & id) {
                            auto robot_pos = world_model.getOurRobot(id)->pose.pos;
                            auto oldest_distance = (records.front().position - robot_pos).norm();
                            if (oldest_distance > threshold) {
                              return false;
                            }
                            for (const auto & record : records) {
                              auto distance = (record.position - robot_pos).norm();
                              if (distance < oldest_distance) {
                                return false;
                              }
                            }
                            return true;
                          }) |
                          ranges::to<std::vector>();

  detected_bots.enemies = available_bots.enemies | filter([&](const auto & id) {
                            auto robot_pos = world_model.getTheirRobot(id)->pose.pos;
                            auto oldest_distance = (records.front().position - robot_pos).norm();
                            if (oldest_distance > threshold) {
                              return false;
                            }
                            for (const auto & record : records) {
                              auto distance = (record.position - robot_pos).norm();
                              if (distance < oldest_distance) {
                                return false;
                              }
                            }
                            return true;
                          }) |
                          ranges::to<std::vector>();
  return detected_bots;
}

auto KickEventDetector::filterByVelocity(
  double threshold, const DetectedBots & available_bots,
  const WorldModelWrapper & world_model) -> DetectedBots
{
  // records内にthresholdより速いボールがあるかどうかを確認する
  auto faster_records = records | ranges::view::filter([&](const auto & record) {
                          return record.velocity.norm() > threshold;
                        }) |
                        ranges::to<std::vector>();

  if (not faster_records.empty()) {
    return available_bots;
  } else {
    return DetectedBots();
  }
}

auto KickEventDetector::filterByBotAngle(
  double threshold, const DetectedBots & available_bots,
  const WorldModelWrapper & world_model) -> DetectedBots
{
  // ロボットの向いている方向にボールがあるかどうかを確認する
  DetectedBots detected_bots;
  detected_bots.friends = available_bots.friends | ranges::views::filter([&](const auto & id) {
                            auto robot_pose = world_model.getOurRobot(id)->pose;
                            auto ball_angle = getAngle(records.front().position - robot_pose.pos);
                            return getAngleDiff(ball_angle, robot_pose.theta) < threshold;
                          }) |
                          ranges::to<std::vector>();
  detected_bots.enemies = available_bots.enemies | ranges::views::filter([&](const auto & id) {
                            auto robot_pose = world_model.getTheirRobot(id)->pose;
                            auto ball_angle = getAngle(records.front().position - robot_pose.pos);
                            return getAngleDiff(ball_angle, robot_pose.theta) < threshold;
                          }) |
                          ranges::to<std::vector>();
  return detected_bots;
}

auto KickEventDetector::filterByDistanceIncrease(
  const DetectedBots & available_bots, const WorldModelWrapper & world_model) -> DetectedBots
{
  // ボールがロボットから遠ざかり続けているかどうかをrecordsをずらしながら確認する
  DetectedBots detected_bots;
  detected_bots.friends = available_bots.friends | ranges::views::filter([&](const auto & id) {
                            auto robot_pose = world_model.getOurRobot(id)->pose;
                            auto distance = (records.front().position - robot_pose.pos).norm();
                            for (const auto & record : records) {
                              auto new_distance = (record.position - robot_pose.pos).norm();
                              if (new_distance < distance) {
                                return false;
                              }
                              distance = new_distance;
                            }
                            return true;
                          }) |
                          ranges::to<std::vector>();
  detected_bots.enemies = available_bots.enemies | ranges::views::filter([&](const auto & id) {
                            auto robot_pose = world_model.getTheirRobot(id)->pose;
                            auto distance = (records.front().position - robot_pose.pos).norm();
                            for (const auto & record : records) {
                              auto new_distance = (record.position - robot_pose.pos).norm();
                              if (new_distance < distance) {
                                return false;
                              }
                              distance = new_distance;
                            }
                            return true;
                          }) |
                          ranges::to<std::vector>();
  return detected_bots;
}
}  // namespace crane
