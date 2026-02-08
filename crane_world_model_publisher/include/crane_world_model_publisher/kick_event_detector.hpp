// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__KICK_EVENT_DETECTOR_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__KICK_EVENT_DETECTOR_HPP_

#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/kick_prediction_tracker.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/kick.hpp>
#include <crane_msgs/msg/kick_prediction_trace.hpp>
#include <crane_msgs/msg/velocity_command.hpp>
#include <crane_msgs/msg/velocity_commands.hpp>
#include <memory>
#include <queue>

namespace crane
{
class KickerModel;

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
    -> void;

  auto getOnGoingKick() -> std::optional<crane_msgs::msg::Kick>;

  auto hasInterruptedOnGoingKick(const WorldModelWrapper & world_model) const -> bool;

  auto setKickerModel(std::shared_ptr<KickerModel> kicker_model) -> void;

  auto updateRobotCommands(const crane_msgs::msg::VelocityCommands & commands) -> void;

  // 一番古いデータがthresholdより近く、それ以外の全てがthresholdより遠いロボットを検出する
  // つまり、ボールが遠ざかっているときにキックイベントを検出する
  auto filterByDistance(
    double threshold, const DetectedBots & available_bots, const WorldModelWrapper & world_model)
    -> DetectedBots;

  auto filterByVelocity(
    double threshold, const DetectedBots & available_bots, const WorldModelWrapper & world_model)
    -> DetectedBots;

  auto filterByBotAngle(
    double threshold, const DetectedBots & available_bots, const WorldModelWrapper & world_model)
    -> DetectedBots;

  auto filterByDistanceIncrease(
    const DetectedBots & available_bots, const WorldModelWrapper & world_model) -> DetectedBots;

  struct Record
  {
    Point position;
    Point velocity;
    rclcpp::Time timestamp;
  };

  struct RobotCommandRecord
  {
    crane_msgs::msg::VelocityCommands commands;
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

  // キック予測トレース管理
  std::optional<crane_msgs::msg::KickPredictionTrace> ongoing_kick_trace_ = std::nullopt;
  Point kick_origin_pos_;                      // キック開始位置を記録
  std::shared_ptr<KickerModel> kicker_model_;  // キック予測用モデル

  // VelocityCommandsリングバッファ
  std::deque<RobotCommandRecord> robot_command_records_;
  static constexpr int COMMAND_QUEUE_SIZE = 30;  // 約1秒分（30Hz想定）

  // 指定ロボットIDの最新コマンドを取得
  auto getLatestCommandForRobot(uint8_t robot_id) const
    -> std::optional<crane_msgs::msg::VelocityCommand>;
};
}  // namespace crane

#endif  // CRANE_WORLD_MODEL_PUBLISHER__KICK_EVENT_DETECTOR_HPP_
