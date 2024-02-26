// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <crane_planner_plugins/planners.hpp>
#include <filesystem>

#include "crane_session_controller/session_controller.hpp"

namespace crane
{
std::shared_ptr<std::unordered_map<uint8_t, RobotRole>> PlannerBase::robot_roles = nullptr;

SessionControllerComponent::SessionControllerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("session_controller", options),
  robot_commands_pub(create_publisher<crane_msgs::msg::RobotCommands>("/control_targets", 1))
{
  robot_roles = std::make_shared<std::unordered_map<uint8_t, RobotRole>>();
  PlannerBase::robot_roles = robot_roles;
  /*
   * 各セッションの設定の読み込み
   */
  using std::filesystem::path;
  auto session_config_dir =
    path(ament_index_cpp::get_package_share_directory("crane_session_controller")) / "config" /
    "play_situation";

  auto load_session_config = [this](const path & config_file) {
    if (config_file.extension() != ".yaml") {
      return;
    } else {
      RCLCPP_INFO(
        get_logger(), "セッション設定を読み込みます : %s", config_file.filename().string().c_str());
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
  using std::filesystem::directory_iterator;
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
  declare_parameter<std::string>("event_config_file_name", "event_config.yaml");
  auto event_config_file_name = get_parameter("event_config_file_name").as_string();

  auto event_config_path =
    path(ament_index_cpp::get_package_share_directory("crane_session_controller")) / "config" /
    event_config_file_name;
  auto event_config = YAML::LoadFile(event_config_path.c_str());
  std::cout << "----------------------------------------" << std::endl;
  for (auto event_node : event_config["events"]) {
    std::cout << "イベント「" << event_node["event"] << "」の設定を読み込みます" << std::endl;
    event_map[event_node["event"].as<std::string>()] = event_node["session"].as<std::string>();
  }

  game_analysis_sub = create_subscription<crane_msgs::msg::GameAnalysis>(
    "/game_analysis", 1, [](const crane_msgs::msg::GameAnalysis & msg) {
      // TODO(HansRobo): 実装
    });

  play_situation_sub = create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 1, [this](const crane_msgs::msg::PlaySituation & msg) {
      // TODO(HansRobo): 実装
      if (not world_model_ready) {
        return;
      }
      play_situation.update(msg);
      auto it = event_map.find(play_situation.getSituationCommandText());
      if (it != event_map.end()) {
        RCLCPP_INFO(
          get_logger(),
          "イベント「%s」に対応するセッション「%"
          "s」の設定に従ってロボットを割り当てます",
          it->first.c_str(), it->second.c_str());
        request(it->second, world_model->ours.getAvailableRobotIds());
      } else {
        RCLCPP_ERROR(
          get_logger(), "イベント「%s」に対応するセッションの設定が見つかりませんでした",
          play_situation.getSituationCommandText().c_str());
      }
    });

  using std::chrono::operator""ms;
  timer = create_wall_timer(100ms, [&]() {
    auto it = event_map.find(play_situation.getSituationCommandText());
    if (it != event_map.end()) {
      //      request(it->second, world_model->ours.getAvailableRobotIds());
    }
  });

  declare_parameter("initial_session", "HALT");
  auto initial_session = get_parameter("initial_session").as_string();

  world_model = std::make_shared<WorldModelWrapper>(*this);

  visualizer = std::make_shared<ConsaiVisualizerWrapper>(*this, "session_controller");

  world_model->addCallback([this, initial_session]() {
    if (not world_model_ready && not world_model->ours.getAvailableRobotIds().empty()) {
      world_model_ready = true;
      auto it = event_map.find(initial_session);
      if (it != event_map.end()) {
        RCLCPP_INFO(
          get_logger(),
          "初期イベント「%s」に対応するセッション「%"
          "s」の設定に従ってロボットを割り当てます",
          it->first.c_str(), it->second.c_str());
        request(it->second, world_model->ours.getAvailableRobotIds());
      } else {
        RCLCPP_ERROR(
          get_logger(), "初期イベント「%s」に対応するセッションの設定が見つかりませんでした",
          initial_session.c_str());
      }
    }
  });

  world_model->addCallback([this]() {
    crane_msgs::msg::RobotCommands msg;
    msg.header = world_model->getMsg().header;
    msg.on_positive_half = world_model->onPositiveHalf();
    msg.is_yellow = world_model->isYellow();
    for (const auto & planner : available_planners) {
      auto commands_msg = planner->getRobotCommands();
      msg.robot_commands.insert(
        msg.robot_commands.end(), commands_msg.robot_commands.begin(),
        commands_msg.robot_commands.end());
      if (planner->getStatus() != PlannerBase::Status::RUNNING) {
        // TODO(HansRobo): プランナが成功・失敗した場合の処理
      }
    }
    robot_commands_pub->publish(msg);
  });
}

void SessionControllerComponent::request(
  const std::string & situation, std::vector<uint8_t> selectable_robot_ids)
{
  RCLCPP_INFO(
    get_logger(), "「%s」というSituationに対してロボット割当を実行します", situation.c_str());
  std::string ids_string;
  for (auto id : selectable_robot_ids) {
    ids_string += std::to_string(id) + " ";
  }
  RCLCPP_INFO(get_logger(), "\t選択可能なロボットID : %s", ids_string.c_str());

  auto map = robot_selection_priority_map.find(situation);
  if (map == robot_selection_priority_map.end()) {
    RCLCPP_ERROR(
      get_logger(),
      "\t「%"
      "s」というSituationに対してロボット割当リクエストが発行されましたが，"
      "見つかりませんでした",
      situation.c_str());
    return;
  }

  auto prev_available_planners =
    std::exchange(available_planners, std::vector<PlannerBase::UniquePtr>());

  // 優先順位が高いPlannerから順にロボットを割り当てる
  for (auto p : map->second) {
    auto req = std::make_shared<crane_msgs::srv::RobotSelect::Request>();
    req->selectable_robots_num = p.selectable_robot_num;
    // 使用可能なロボットを詰め込む
    for (auto id : selectable_robot_ids) {
      req->selectable_robots.emplace_back(id);
    }
    try {
      auto response = [&p, &req, this]() {
        auto planner = generatePlanner(p.session_name, world_model, visualizer);
        auto response = planner->doRobotSelect(req);
        available_planners.emplace_back(std::move(planner));
        return response;
      }();

      // 前回結果との比較
      for (auto & prev_planner : prev_available_planners) {
        if (prev_planner->isSameConfiguration(prev_planner.get())) {
          RCLCPP_INFO(
            get_logger(), "\t前回と同じ割当結果が得られたため，前回のプランナを再利用します");
          available_planners.pop_back();
          available_planners.emplace_back(std::move(prev_planner));
          break;
        }
      }

      // 割当依頼結果の反映
      std::string id_list_string;
      for (auto id : response.selected_robots) {
        id_list_string += std::to_string(id) + " ";
      }
      RCLCPP_INFO(
        get_logger(), "\tセッション「%s」に以下のロボットを割り当てました : %s",
        p.session_name.c_str(), id_list_string.c_str());
      for (auto selected_robot_id : response.selected_robots) {
        // 割当されたロボットを利用可能ロボットリストから削除
        selectable_robot_ids.erase(
          remove(selectable_robot_ids.begin(), selectable_robot_ids.end(), selected_robot_id),
          selectable_robot_ids.end());
        // 割当されたロボットをロールマップに追加(この情報は他のプランナにも共有される)
        robot_roles->insert_or_assign(selected_robot_id, RobotRole{p.session_name, ""});
      }
    } catch (std::exception & e) {
      RCLCPP_ERROR(
        get_logger(), "\t「%s」というプランナを呼び出した時に例外が発生しました : %s",
        p.session_name.c_str(), e.what());
      break;
    }
  }
  // TODO(HansRobo): 割当が終わっても無職のロボットは待機状態にする
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::SessionControllerComponent)
