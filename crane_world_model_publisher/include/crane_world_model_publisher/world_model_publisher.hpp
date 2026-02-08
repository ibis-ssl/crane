// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_PUBLISHER_HPP_
#define CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_PUBLISHER_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CRANE_EXPORT __attribute__((dllexport))
#define CRANE_IMPORT __attribute__((dllimport))
#else
#define CRANE_EXPORT __declspec(dllexport)
#define CRANE_IMPORT __declspec(dllimport)
#endif
#ifdef CRANE_BUILDING_DLL
#define CRANE_PUBLIC CRANE_EXPORT
#else
#define CRANE_PUBLIC CRANE_IMPORT
#endif
#define CRANE_PUBLIC_TYPE CRANE_PUBLIC
#define CRANE_LOCAL
#else
#define CRANE_EXPORT __attribute__((visibility("default")))
#define CRANE_IMPORT
#if __GNUC__ >= 4
#define CRANE_PUBLIC __attribute__((visibility("default")))
#define CRANE_LOCAL __attribute__((visibility("hidden")))
#else
#define CRANE_PUBLIC
#define CRANE_LOCAL
#endif
#define CRANE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#include <array>
#include <crane_comm/diagnosed_publisher.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <crane_msgs/msg/velocity_commands.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <deque>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace crane
{
enum class Color : uint8_t {
  BLUE,
  YELLOW,
};

class WorldModelDataProvider;
class VisualizationManager;
class KickEventDetector;
class PassTargetSelector;
class WorldModelWrapper;

class WorldModelPublisherComponent : public rclcpp::Node
{
public:
  CRANE_PUBLIC
  explicit WorldModelPublisherComponent(const rclcpp::NodeOptions &);
  CRANE_PUBLIC
  ~WorldModelPublisherComponent() override;

private:
  using WorldModelWrapperPtr = std::shared_ptr<WorldModelWrapper>;

  auto publishWorldModel() -> void;

  auto publishVisualization(WorldModelWrapperPtr world_model) -> void;

  auto updateHistory(crane_msgs::msg::WorldModel & msg) -> void;

  auto postProcessWorldModel(WorldModelWrapperPtr) -> void;

  auto updateBallContact() -> void;

  auto updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) -> void;

  std::unique_ptr<WorldModelDataProvider> data_provider_;

  static constexpr float DISAPPEARED_TIME_THRESH = 3.0f;

  diagnostic_updater::Updater diagnostic_updater_;

  DiagnosedPublisher<crane_msgs::msg::WorldModel> pub_world_model;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_process_time;

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<VisualizationManager> visualization_manager_;

  std::array<std::deque<crane_msgs::msg::RobotInfo>, 20> friend_history;

  std::array<std::deque<crane_msgs::msg::RobotInfo>, 20> enemy_history;

  std::deque<crane_msgs::msg::BallInfo> ball_info_history;

  int history_size{};

  enum class BallEvent {
    NONE,
    OUR_BALL,
    THEIR_BALL,
  } last_ball_event = BallEvent::NONE;

  WorldModelWrapperPtr wrapper_;

  std::unique_ptr<KickEventDetector> kick_event_detector_;

  std::unique_ptr<PassTargetSelector> pass_target_selector_;

  rclcpp::Subscription<crane_msgs::msg::VelocityCommands>::SharedPtr sub_robot_commands_;
};
}  // namespace crane
#endif  // CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_PUBLISHER_HPP_
