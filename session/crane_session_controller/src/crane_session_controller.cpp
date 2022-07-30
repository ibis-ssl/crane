// Copyright (c) 2022 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "crane_session_controller/session_controller.hpp"

namespace crane
{

SessionControllerComponent::SessionControllerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("session_controller", options)
{
  // example of adding planner
  // session_planners_["replace"] = std::make_shared<SessionModule>("replace");
  session_planners_["waiter"] = std::make_shared<SessionModule>("waiter");
  session_planners_["goalie"] = std::make_shared<SessionModule>("goalie");
  session_planners_["defender"] = std::make_shared<SessionModule>("defender");
  for (auto & planner : session_planners_) {
    planner.second->construct(*this);
  }

  // example for ball replacement
  // TODO : load from config file
  // TODO:
  auto defense_map = std::vector<SessionCapacity>();
  defense_map.emplace_back(SessionCapacity({"goalie", 1}));
  defense_map.emplace_back(SessionCapacity({"defender", 2}));
  defense_map.emplace_back(SessionCapacity({"waiter", 100}));
  robot_selection_priority_map["defense"] = defense_map;

//  auto test_map = std::vector<SessionCapacity>();
//  test_map.emplace_back(SessionCapacity({"waiter", 100}));
//  robot_selection_priority_map["test"] = test_map;

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(1s, std::bind(&SessionControllerComponent::timerCallback, this));

  world_model_ = std::make_shared<WorldModelWrapper>(*this);

  // expect : {goalie : 1, defender : 2, waiter : 1}
  request("defense", {0, 1, 2, 3});
}

void SessionControllerComponent::request(
  std::string situation, std::vector<int> selectable_robot_ids)
{
  RCLCPP_INFO(get_logger(), "request : %s", situation.c_str());
  std::string ids_string;
  for (auto id : selectable_robot_ids) {
    ids_string += std::to_string(id) + " ";
  }
  RCLCPP_INFO(get_logger(), "selectable_robot_ids : %s", ids_string.c_str());

  auto map = robot_selection_priority_map.find(situation);
  if (map == robot_selection_priority_map.end()) {
    RCLCPP_ERROR(get_logger(), "no such situation : %s", situation.c_str());
    return;
  }

  for (auto p : map->second) {
    auto req = std::make_shared<crane_msgs::srv::RobotSelect::Request>();
    req->selectable_robots_num = p.selectable_robot_num;
    for (auto id : selectable_robot_ids) {
      req->selectable_robots.emplace_back(id);
    }
    try {
      auto planner = session_planners_.find(p.session_name);
      if (planner == session_planners_.end()) {
        RCLCPP_ERROR(get_logger(), "Session planner is not found : %s", p.session_name.c_str());
        break;
      }
      auto response = planner->second->sendRequest(req, *this);
      if (!response) {
        RCLCPP_ERROR(
          get_logger(), "Failed to get response from the session planner : %s",
          p.session_name.c_str());
        break;
      }

      std::string ids_string;
      for (auto id : response->selected_robots) {
        ids_string += std::to_string(id) + " ";
      }
      RCLCPP_INFO(
        get_logger(), "Assigned to [%s] : %s", p.session_name.c_str(), ids_string.c_str());
      for (auto selected_robot_id : response->selected_robots) {
        // delete selected robot from available robot list
        selectable_robot_ids.erase(
          remove(selectable_robot_ids.begin(), selectable_robot_ids.end(), selected_robot_id),
          selectable_robot_ids.end());
      }
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Undefined session planner is called : %s", p.session_name.c_str());
      break;
    }
  }
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(crane::SessionControllerComponent)
