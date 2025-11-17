// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
#define CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_

#include <chrono>
#include <crane_comm/diagnosed_publisher.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/play_situation_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_select_results.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_plugins/planner_base.hpp>
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

#include "visibility_control.h"

namespace crane
{
struct SessionCapacity
{
  std::string session_name;

  int selectable_robot_num;
};

class SessionControllerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit SessionControllerComponent(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  auto request(
    const std::string & situation, std::vector<uint8_t> selectable_robot_ids, PlannerContext &)
    -> void;

  auto assign(const std::string & event_name) -> void;

private:
  auto updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) -> void;

  WorldModelWrapper::SharedPtr world_model;

  std::deque<crane_msgs::srv::RobotSelect::Request> query_queue;

  //  identifier: situation name,
  //    content: [ list of  [ pair of session name & selectable robot num]]
  std::unordered_map<std::string, std::vector<SessionCapacity>> robot_selection_priority_map;

  //  identifier :  event name, content : situation name
  std::unordered_map<std::string, std::string> event_map;

  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_sub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr session_injection_sub;

  DiagnosedPublisher<crane_msgs::msg::RobotCommands> robot_commands_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr timer_process_time_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr callback_process_time_pub;

  rclcpp::Publisher<crane_msgs::msg::RobotSelectResults>::SharedPtr robot_select_results_pub;

  std::vector<PlannerBase::SharedPtr> available_planners;

  crane_msgs::msg::PlaySituation play_situation;

  rclcpp::TimerBase::SharedPtr timer;

  bool world_model_ready = false;

  std::shared_ptr<std::unordered_map<uint8_t, RobotRole>> robot_roles;

  VisualizerMessageBuilder::SharedPtr visualizer =
    std::make_shared<VisualizerMessageBuilder>("session_controller");

  diagnostic_updater::Updater diagnostic_updater_;

  rclcpp::Time last_planning_time_;

  int planning_count_ = 0;

  std::string prev_session_name_;

  std::string prev_assignment_log_;
};

}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
