// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__KICK_EVENT_DETECTOR_HPP_
#define CRANE_GAME_ANALYZER__KICK_EVENT_DETECTOR_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <queue>

namespace crane
{
struct DetectedBots
{
  std::vector<uint8_t> friends;
  std::vector<uint8_t> enemies;
};

struct KickOrigin
{
  rclcpp::Time stamp;
  Point position;
  RobotIdentifier robot;
};

class KickEventDetector
{
public:
  KickEventDetector() : ros_clock(RCL_ROS_TIME) {}

  void update(
    const WorldModelWrapper::UniquePtr & world_model,
    const ConsaiVisualizerBuffer::MessageBuilder::UniquePtr & visualizer)
  {
    {
      Record record;
      record.position = world_model->ball.pos;
      record.velocity = world_model->ball.vel;
      records.push_back(record);
    }

    if (records.size() > QUEUE_SIZE) {
      records.pop_front();
    } else {
      return;
    }

    DetectedBots available_bots;
    available_bots.friends = world_model->ours.getAvailableRobotIds();
    available_bots.enemies = world_model->theirs.getAvailableRobotIds();

    auto detected_bots = filterByDistance(distance_threshold, available_bots, world_model);
    detected_bots = filterByVelocity(0.5, detected_bots, world_model);
    detected_bots = filterByBotAngle(0.5, detected_bots, world_model);
    detected_bots = filterByDistanceIncrease(detected_bots, world_model);
    // print detected bots
    std::optional<KickOrigin> kick_event_origin = std::nullopt;
    for (const auto & id : detected_bots.friends) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("aaaa"), "Detected friend: " << static_cast<int>(id));
      visualizer->addCircle(
        world_model->getOurRobot(id)->pose.pos, 0.5, 2, "blue", "blue", 1.0, "KICK");
      kick_event_origin.emplace(ros_clock.now(), world_model->ball.pos, RobotIdentifier{true, id});
    }
    for (const auto & id : detected_bots.enemies) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("aaaa"), "Detected enemy: " << static_cast<int>(id));
      visualizer->addCircle(
        world_model->getTheirRobot(id)->pose.pos, 0.5, 2, "blue", "blue", 1.0, "KICK");
      kick_event_origin.emplace(ros_clock.now(), world_model->ball.pos, RobotIdentifier{false, id});
    }

    // 進行中キックの更新
    if (kick_event_origin.has_value()) {
      ongoing_kick_origin = kick_event_origin.value();
    } else {
      if (ongoing_kick_origin.has_value() && hasInterruptedOnGoingKick(world_model)) {
        // キック中断判定
        kick_history.push_back({ongoing_kick_origin.value(), world_model->ball.pos});
        ongoing_kick_origin = std::nullopt;
        visualizer->addCircle(world_model->ball.pos, 3.5, 2, "green", "black", 1.0, "EVENT");
      }
    }

    // 進行中のキックを可視化
    if (ongoing_kick_origin.has_value()) {
      visualizer->addTube(
        world_model->ball.pos, ongoing_kick_origin.value().position, 0.2, 2, "red", "", 1.0,
        "KICK");
    }

    // ボールの履歴を可視化
    for (const auto & kick : kick_history) {
      visualizer->addLine(kick.first.position, kick.second, 2, "red", 0.5, "KICK");
    }

    for (const auto & record : records) {
      visualizer->addCircle(record.position, 0.1, 2, "red", "", 1.0, "ball");
    }
  }

  bool hasInterruptedOnGoingKick(const WorldModelWrapper::UniquePtr & world_model) const
  {
    const auto & latest = records.back();
    const auto & pre = records.at(records.size() - 2);
    double pre_vel = pre.velocity.norm();
    double vel_diff = (latest.velocity - pre.velocity).norm();

    double score = vel_diff / (pre_vel + 0.1) * 100;
    bool event_detected = score > 30;

    return world_model->ball.isStopped(0.5) or event_detected;
  }

  // 一番古いデータがthresholdより近く、それ以外の全てがthresholdより遠いロボットを検出する
  // つまり、ボールが遠ざかっているときにキックイベントを検出する
  DetectedBots filterByDistance(
    double threshold, const DetectedBots & available_bots,
    const WorldModelWrapper::UniquePtr & world_model)
  {
    using ranges::views::filter;
    DetectedBots detected_bots;
    detected_bots.friends = available_bots.friends | filter([&](const auto & id) {
                              auto robot_pos = world_model->getOurRobot(id)->pose.pos;
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
                              auto robot_pos = world_model->getTheirRobot(id)->pose.pos;
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

  DetectedBots filterByVelocity(
    double threshold, const DetectedBots & available_bots,
    const WorldModelWrapper::UniquePtr & world_model)
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

  DetectedBots filterByBotAngle(
    double threshold, const DetectedBots & available_bots,
    const WorldModelWrapper::UniquePtr & world_model)
  {
    // ロボットの向いている方向にボールがあるかどうかを確認する
    DetectedBots detected_bots;
    detected_bots.friends = available_bots.friends | ranges::views::filter([&](const auto & id) {
                              auto robot_pose = world_model->getOurRobot(id)->pose;
                              auto ball_angle = getAngle(records.front().position - robot_pose.pos);
                              return getAngleDiff(ball_angle, robot_pose.theta) < threshold;
                            }) |
                            ranges::to<std::vector>();
    detected_bots.enemies = available_bots.enemies | ranges::views::filter([&](const auto & id) {
                              auto robot_pose = world_model->getTheirRobot(id)->pose;
                              auto ball_angle = getAngle(records.front().position - robot_pose.pos);
                              return getAngleDiff(ball_angle, robot_pose.theta) < threshold;
                            }) |
                            ranges::to<std::vector>();
    return detected_bots;
  }

  DetectedBots filterByDistanceIncrease(
    const DetectedBots & available_bots, const WorldModelWrapper::UniquePtr & world_model)
  {
    // ボールがロボットから遠ざかり続けているかどうかをrecordsをずらしながら確認する
    DetectedBots detected_bots;
    detected_bots.friends = available_bots.friends | ranges::views::filter([&](const auto & id) {
                              auto robot_pose = world_model->getOurRobot(id)->pose;
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
                              auto robot_pose = world_model->getTheirRobot(id)->pose;
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

  struct Record
  {
    Point position;
    Point velocity;
    rclcpp::Time timestamp;
  };

private:
  std::deque<Record> records;

  std::optional<KickOrigin> ongoing_kick_origin = std::nullopt;

  std::deque<std::pair<KickOrigin, Point>> kick_history;

  static constexpr int QUEUE_SIZE = 5;

  double distance_threshold = 0.15;

  rclcpp::Clock ros_clock;
};
}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__KICK_EVENT_DETECTOR_HPP_
