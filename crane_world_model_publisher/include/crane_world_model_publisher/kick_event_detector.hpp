// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__KICK_EVENT_DETECTOR_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__KICK_EVENT_DETECTOR_HPP_

#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/kick.hpp>
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
  KickEventDetector()
  : ros_clock(RCL_ROS_TIME), visualizer(std::make_shared<VisualizerMessageBuilder>("kick_event"))
  {
  }

  auto update(
    const WorldModelWrapper & world_model, const VisualizerMessageBuilder::SharedPtr visualizer)
    -> void
  ;

  auto getOnGoingKick() -> std::optional<crane_msgs::msg::Kick>
  ;

  auto hasInterruptedOnGoingKick(const WorldModelWrapper & world_model) const -> bool
  ;

  // 一番古いデータがthresholdより近く、それ以外の全てがthresholdより遠いロボットを検出する
  // つまり、ボールが遠ざかっているときにキックイベントを検出する
  auto filterByDistance(
    double threshold, const DetectedBots & available_bots, const WorldModelWrapper & world_model)
    -> DetectedBots
  ;

  auto filterByVelocity(
    double threshold, const DetectedBots & available_bots, const WorldModelWrapper & world_model)
    -> DetectedBots
  ;

  auto filterByBotAngle(
    double threshold, const DetectedBots & available_bots, const WorldModelWrapper & world_model)
    -> DetectedBots
  ;

  auto filterByDistanceIncrease(
    const DetectedBots & available_bots, const WorldModelWrapper & world_model) -> DetectedBots
  ;

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

  VisualizerMessageBuilder::SharedPtr visualizer;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__KICK_EVENT_DETECTOR_HPP_
