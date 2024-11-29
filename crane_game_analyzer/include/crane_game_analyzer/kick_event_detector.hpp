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
class KickEventDetector
{
public:
  void update(const WorldModelWrapper::SharedPtr & world_model)
  {
    Record record;
    record.position = world_model->ball.pos;
    record.velocity = world_model->ball.vel;
    records.push(record);
    if (records.size() > QUEUE_SIZE) {
      records.pop();
    }

    DetectedBots available_bots;
    available_bots.friends = world_model->ours.getAvailableRobotIds();
    available_bots.enemies = world_model->theirs.getAvailableRobotIds();

    auto detected_bots = validateByDistance(distance_threshold, available_bots, world_model);
    detected_bots = validateByVelocity(0.5, detected_bots, world_model);
    detected_bots = validateByBotAngle(0.5, detected_bots, world_model);
    detected_bots = validateByDistanceIncrease(detected_bots, world_model);
    // print detected bots
    for (const auto & id : detected_bots.friends) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("aaaa"), "Detected friend: " << static_cast<int>(id));
    }
    for (const auto & id : detected_bots.enemies) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("aaaa"), "Detected enemy: " << static_cast<int>(id));
    }
  }

  // 一番古いデータがthreshouldeより近く、それ以外の全てがthresholdより遠いロボットを検出する
  // つまり、ボールが遠ざかっているときにキックイベントを検出する
  DetectedBots validateByDistance(
    double threshold, const DetectedBots & available_bots,
    const WorldModelWrapper::SharedPtr & world_model)
  {
    using ranges::views::filter;
    DetectedBots detected_bots;
    detected_bots.friends =
      available_bots.friends | filter([&](const auto & id) {
        auto distance = (records.front().position - world_model->getOurRobot(id)->pose.pos).norm();
        return distance < threshold;
      }) |
      ranges::to<std::vector>();
    detected_bots.enemies =
      available_bots.enemies | filter([&](const auto & id) {
        auto distance =
          (records.front().position - world_model->getTheirRobot(id)->pose.pos).norm();
        return distance < threshold;
      }) |
      ranges::to<std::vector>();
    return detected_bots;
  }

  DetectedBots validateByVelocity(
    double threshold, const DetectedBots & available_bots,
    const WorldModelWrapper::SharedPtr & world_model)
  {
    // records内にthresholdより速いボールがあるかどうかを確認する
    auto faster_records = records | ranges::view::filter([&](const auto & record) {
                            return record.velocity.norm() > threshold;
                          }) |
                          ranges::to<std::vector>();

    if (faster_records.empty()) {
      return available_bots;
    } else {
      return DetectedBots();
    }
  }

  DetectedBots validateByBotAngle(
    double threshold, const DetectedBots & available_bots,
    const WorldModelWrapper::SharedPtr & world_model)
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

  DetectedBots validateByDistanceIncrease(
    const DetectedBots & available_bots, const WorldModelWrapper::SharedPtr & world_model)
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
  std::queue<Record> records;
  static constexpr int QUEUE_SIZE = 10;

  double distance_threshold = 0.15;
};
}  // namespace crane

#endif  // CRANE_GAME_ANALYZER__KICK_EVENT_DETECTOR_HPP_
