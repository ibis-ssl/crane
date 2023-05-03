// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
  auto defense_map = std::vector<SessionCapacity>();
  defense_map.emplace_back(SessionCapacity({"goalie", 1}));
  defense_map.emplace_back(SessionCapacity({"defender", 5}));
  defense_map.emplace_back(SessionCapacity({"waiter", 100}));
  robot_selection_priority_map["defense"] = defense_map;

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(1s, std::bind(&SessionControllerComponent::timerCallback, this));

  world_model_ = std::make_shared<WorldModelWrapper>(*this);

  // expect : {goalie : 1, defender : 2, waiter : 1}
  request("defense", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
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

  // 優先順位が高いPlannerから順にロボットを割り当てる
  for (auto p : map->second) {
    auto req = std::make_shared<crane_msgs::srv::RobotSelect::Request>();
    req->selectable_robots_num = p.selectable_robot_num;
    // 使用可能なロボットを詰め込む
    for (auto id : selectable_robot_ids) {
      req->selectable_robots.emplace_back(id);
    }
    try {
      auto planner = session_planners_.find(p.session_name);
      if (planner == session_planners_.end()) {
        RCLCPP_ERROR(get_logger(), "Session planner is not found : %s", p.session_name.c_str());
        break;
      }
      // plannerにロボット割り当てを依頼する
      auto response = planner->second->sendRequest(req, *this);
      if (!response) {
        RCLCPP_ERROR(
          get_logger(), "Failed to get response from the session planner : %s",
          p.session_name.c_str());
        break;
      }

      // 割当依頼結果の反映
      std::string ids_string;
      for (auto id : response->selected_robots) {
        ids_string += std::to_string(id) + " ";
      }
      RCLCPP_INFO(
        get_logger(), "Assigned to [%s] : %s", p.session_name.c_str(), ids_string.c_str());
      for (auto selected_robot_id : response->selected_robots) {
        // 割当されたロボットを利用可能ロボットリストから削除
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
  // TODO：割当が終わっても無職のロボットは待機状態にする
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::SessionControllerComponent)
