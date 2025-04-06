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

#include <crane_basics/diagnosed_publisher.hpp>
#include <crane_basics/multicast.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_msgs/msg/robot_info.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_world_model_publisher/kick_event_detector.hpp>
#include <crane_world_model_publisher/world_model_data_provider.hpp>
#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace crane
{
enum class Color : uint8_t {
  BLUE,
  YELLOW,
};

class WorldModelPublisherComponent : public rclcpp::Node
{
public:
  CRANE_PUBLIC
  explicit WorldModelPublisherComponent(const rclcpp::NodeOptions &);

private:
  void publishWorldModel();

  void publishVisualization();

  void updateHistory(crane_msgs::msg::WorldModel & msg);

  void postProcessWorldModel(WorldModelWrapper::SharedPtr);

  void updateBallContact();

  WorldModelDataProvider data_provider;

  static constexpr float DISAPPEARED_TIME_THRESH = 3.0f;

  DiagnosedPublisher<crane_msgs::msg::WorldModel> pub_world_model;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_process_time;

  rclcpp::TimerBase::SharedPtr timer;

  VisualizerMessageBuilder::SharedPtr visualizer;

  VisualizerMessageBuilder::SharedPtr pass_score_visualizer;

  std::array<std::deque<crane_msgs::msg::RobotInfo>, 20> friend_history;

  std::array<std::deque<crane_msgs::msg::RobotInfo>, 20> enemy_history;

  std::deque<crane_msgs::msg::BallInfo> ball_info_history;

  int history_size;

  enum class BallEvent {
    NONE,
    OUR_BALL,
    THEIR_BALL,
  } last_ball_event = BallEvent::NONE;

  WorldModelWrapper::SharedPtr wrapper;

  KickEventDetector kick_event_detector;
};
}  // namespace crane
#endif  // CRANE_WORLD_MODEL_PUBLISHER__WORLD_MODEL_PUBLISHER_HPP_
