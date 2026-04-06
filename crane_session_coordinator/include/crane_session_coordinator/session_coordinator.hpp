// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_COORDINATOR__SESSION_COORDINATOR_HPP_
#define CRANE_SESSION_COORDINATOR__SESSION_COORDINATOR_HPP_

#include <chrono>
#include <crane_comm/diagnosed_publisher.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/play_situation_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/game_analysis.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/practice_mode.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_select_results.hpp>
#include <crane_sessions/session_base.hpp>
#include <deque>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "command_aggregator.hpp"
#include "configuration_manager.hpp"
#include "diagnostics_reporter.hpp"
#include "robot_allocator.hpp"
#include "session_registry.hpp"
#include "visibility_control.h"

namespace crane
{

class SessionCoordinatorComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit SessionCoordinatorComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  auto request(const std::string & situation, std::vector<uint8_t> selectable_robot_ids) -> void;

  auto assign(const std::string & event_name) -> void;

private:
  auto updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) -> void;

  auto onWorldModelUpdate() -> void;

  WorldModelWrapper::SharedPtr world_model;

  std::shared_ptr<ConfigurationManager> config_manager_;

  std::shared_ptr<SessionRegistry> session_registry_;

  std::unique_ptr<DiagnosticsReporter> diagnostics_reporter_;

  std::unique_ptr<CommandAggregator> command_aggregator_;

  std::unique_ptr<RobotAllocator> robot_allocator_;

  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_sub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr session_injection_sub;

  rclcpp::Subscription<crane_msgs::msg::GameAnalysis>::SharedPtr game_analysis_sub;

  DiagnosedPublisher<crane_msgs::msg::RobotCommands> position_commands_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr timer_process_time_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr callback_process_time_pub;

  rclcpp::Publisher<crane_msgs::msg::RobotSelectResults>::SharedPtr robot_select_results_pub;

  rclcpp::Publisher<crane_msgs::msg::PracticeMode>::SharedPtr practice_mode_pub;

  crane_msgs::msg::PlaySituation play_situation;

  crane_msgs::msg::GameAnalysis latest_game_analysis_;

  rclcpp::TimerBase::SharedPtr timer;

  bool world_model_ready = false;

  bool initial_assignment_done = false;

  std::string initial_session_name;

  std::shared_ptr<std::unordered_map<uint8_t, RobotRole>> robot_roles;

  VisualizerMessageBuilder::SharedPtr visualizer =
    std::make_shared<VisualizerMessageBuilder>("coordinator");

  diagnostic_updater::Updater diagnostic_updater_;

  std::string prev_session_name_;
};

}  // namespace crane
#endif  // CRANE_SESSION_COORDINATOR__SESSION_COORDINATOR_HPP_
