// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
#define CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/game_analysis.hpp"
#include "crane_msgs/msg/play_situation.hpp"
#include "crane_msgs/srv/robot_select.hpp"
#include "crane_session_controller/session_module.hpp"
#include "crane_session_controller/visibility_control.h"

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

  void request(std::string situation, std::vector<int> selectable_robot_ids);

private:
  WorldModelWrapper::SharedPtr world_model;

  std::deque<crane_msgs::srv::RobotSelect::Request> query_queue;

  rclcpp::Client<crane_msgs::srv::RobotSelect>::SharedPtr robot_select_client;

  std::unordered_map<std::string, SessionModule::SharedPtr> session_planners;

  //  identifier :  situation name,  content :   [ list of  [ pair of session name & selectable robot num]]
  std::unordered_map<std::string, std::vector<SessionCapacity>> robot_selection_priority_map;

  rclcpp::Subscription<crane_msgs::msg::GameAnalysis>::SharedPtr game_analysis_sub;

  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_sub;
};

}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__SESSION_CONTROLLER_HPP_
