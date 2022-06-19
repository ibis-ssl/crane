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
  session_planners_["replace"] = std::make_shared<SessionModule>("replace");
  for (auto & planner : session_planners_) {
    planner.second->construct(*this);
  }

  // example for ball replacement
  // TODO : load from config file
  // TODO:
  auto replace_map = std::make_shared<std::vector<SessionCapacity>>();
  replace_map->emplace_back(SessionCapacity({"goalie", 1}));
  replace_map->emplace_back(SessionCapacity({"replace", 2}));
  replace_map->emplace_back(SessionCapacity({"waiter", 100}));
  robot_selection_priority_map["ball_replacement"] = replace_map;

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(1s, std::bind(&SessionControllerComponent::timerCallback, this));

  world_model_ = std::make_shared<WorldModelWrapper>(*this);
}

void SessionControllerComponent::request(
  std::string situation, std::vector<int> selectable_robot_ids)
{
  auto map = robot_selection_priority_map[situation];
  if (!map) {
    RCLCPP_ERROR(get_logger(), "Undefined session module is called : ", situation.c_str());
    return;
  }

  for (auto p : *map) {
    auto req = std::make_shared<crane_msgs::srv::RobotSelect::Request>();
    req->selectable_robots_num = p.selectable_robot_num;
    for (auto id : selectable_robot_ids) {
      req->selectable_robots.emplace_back(id);
    }
    try {
      auto response = session_planners_[p.session_name]->sendRequest(req,shared_from_this());
      if (!response) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to get response from the session planner : ", p.session_name.c_str());
        break;
      }

      for (auto selected_robot_id : response->selected_robots) {
        // delete selected robot from available robot list
        selectable_robot_ids.erase(
          remove(selectable_robot_ids.begin(), selectable_robot_ids.end(), selected_robot_id),
          selectable_robot_ids.end());
      }
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Undefined session planner is called : ", p.session_name.c_str());
      break;
    }
  }
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(crane::SessionControllerComponent)
