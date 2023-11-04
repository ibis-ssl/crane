// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
#define CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_

#include <chrono>
#include <crane_msg_wrappers/play_situation_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/game_analysis.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <deque>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
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
  explicit SessionControllerComponent(const rclcpp::NodeOptions & options);

  void testAssignRequest()
  {
    // expect : {goalie : 1}, {replace : 2}, {waiter : 1}
    request("replace", {0, 1, 2, 3});
  }

  void request(std::string situation, std::vector<uint8_t> selectable_robot_ids);

private:
  WorldModelWrapper::SharedPtr world_model;

  std::deque<crane_msgs::srv::RobotSelect::Request> query_queue;

  //  identifier :  situation name,  content :   [ list of  [ pair of session name & selectable robot num]]
  std::unordered_map<std::string, std::vector<SessionCapacity>> robot_selection_priority_map;

  //  identifier :  event name, content : situation name
  std::unordered_map<std::string, std::string> event_map;

  rclcpp::Subscription<crane_msgs::msg::GameAnalysis>::SharedPtr game_analysis_sub;

  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_sub;

  rclcpp::Publisher<crane_msgs::msg::RobotCommands>::SharedPtr robot_commands_pub;

  std::vector<PlannerBase::UniquePtr> available_planners;

  PlaySituationWrapper play_situation;

  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
