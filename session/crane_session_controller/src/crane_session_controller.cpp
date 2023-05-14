// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include "crane_msg_wrappers/play_situation_wrapper.hpp"
#include "crane_session_controller/session_controller.hpp"

namespace crane
{
SessionControllerComponent::SessionControllerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("session_controller", options)
{
  // example of adding planner
  // session_planners["replace"] = std::make_shared<SessionModule>("replace");
  session_planners["waiter"] = std::make_shared<SessionModule>("waiter");
  session_planners["goalie"] = std::make_shared<SessionModule>("goalie");
  session_planners["defender"] = std::make_shared<SessionModule>("defender");
  for (auto & planner : session_planners) {
    planner.second->construct(*this);
  }

  /*
   * 各セッションの設定の読み込み
   */
  using namespace std::filesystem;
  auto session_config_dir =
    path(ament_index_cpp::get_package_share_directory("crane_session_controller")) / "config" /
    "play_situation";

  auto load_session_config = [this](const path & config_file) {
    if (config_file.extension() != ".yaml") {
      return;
    } else {
      RCLCPP_INFO(get_logger(), "load config : %s", config_file.c_str());
      auto config = YAML::LoadFile(config_file.c_str());
      std::cout << "NAME : " << config["name"] << std::endl;
      std::cout << "DESCRIPTION : " << config["description"] << std::endl;
      std::cout << "SESSIONS : " << std::endl;

      std::vector<SessionCapacity> session_capacity_list;
      for (auto session_node : config["sessions"]) {
        std::cout << "\tNAME     : " << session_node["name"] << std::endl;
        std::cout << "\tCAPACITY : " << session_node["capacity"] << std::endl;
        session_capacity_list.emplace_back(SessionCapacity(
          {session_node["name"].as<std::string>(), session_node["capacity"].as<int>()}));
      }
      robot_selection_priority_map[config["name"].as<std::string>()] = session_capacity_list;
      std::cout << "----------------------------------------" << std::endl;
    }
  };

  std::cout << "----------------------------------------" << std::endl;
  for (auto & path : directory_iterator(session_config_dir)) {
    if (path.is_directory()) {
      for (auto & sub_path : directory_iterator(path.path())) {
        load_session_config(sub_path);
      }
    } else {
      load_session_config(path);
    }
  }

  /*
   * レフェリーイベントとセッションの設定の紐付け
   */
  auto event_config_path =
    path(ament_index_cpp::get_package_share_directory("crane_session_controller")) / "config" /
    "event_config.yaml";
  auto event_config = YAML::LoadFile(event_config_path.c_str());
  std::cout << "----------------------------------------" << std::endl;
  for (auto event_node : event_config["events"]) {
    std::cout << "Load event : " << event_node["event"] << std::endl;
    event_map[event_node["event"].as<std::string>()] = event_node["session"].as<std::string>();
  }

  game_analysis_sub = create_subscription<crane_msgs::msg::GameAnalysis>(
    "/game_analysis", 1, [this](const crane_msgs::msg::GameAnalysis & msg) {
      // TODO
    });

  play_situation_sub = create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 1, [this](const crane_msgs::msg::PlaySituation & msg) {
      // TODO
      PlaySituationWrapper play_situation;
      play_situation.update(msg);
      auto it = event_map.find(play_situation.getSituationCommandText());
      if (it != event_map.end()) {
        RCLCPP_INFO(
          get_logger(),
          "イベント「%s」に対応するセッション「%s」の設定に従ってロボットを割り当てます",
          it->first.c_str(), it->second.c_str());
        // TODO: 選択可能なロボットを引っ張ってくる
        request(it->second, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
      } else {
        RCLCPP_ERROR(
          get_logger(), "イベント「%s」に対応するセッションの設定が見つかりませんでした",
          play_situation.getSituationCommandText().c_str());
      }
    });

  world_model = std::make_shared<WorldModelWrapper>(*this);
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
      auto planner = session_planners.find(p.session_name);
      if (planner == session_planners.end()) {
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
