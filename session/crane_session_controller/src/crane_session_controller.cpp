// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/stacktrace.hpp>
#include <crane_msg_wrappers/delay_monitor_wrapper.hpp>
#include <crane_planner_plugins/planners.hpp>
#include <crane_utils/stream.hpp>
#include <crane_utils/time.hpp>
#include <filesystem>
#include <fstream>
#include <range/v3/all.hpp>
#include <std_msgs/msg/string.hpp>

#include "crane_session_controller/configuration_manager.hpp"
#include "crane_session_controller/planner_registry.hpp"
#include "crane_session_controller/session_controller.hpp"

namespace crane
{
std::shared_ptr<std::unordered_map<uint8_t, RobotRole>> PlannerBase::robot_roles = nullptr;

SessionControllerComponent::SessionControllerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("session_controller", options),
  world_model(std::make_shared<WorldModelWrapper>(*this)),
  robot_commands_pub(this, "/control_targets", 1, 50., 70.),
  robot_select_results_pub(
    create_publisher<crane_msgs::msg::RobotSelectResults>("/robot_select_results", 10)),
  diagnostic_updater_(this),
  last_planning_time_(this->now())
{
  crane::CraneVisualizerBuffer::activate(*this);

  world_model->setBallOwnerCalculatorEnabled(true);
  robot_roles = std::make_shared<std::unordered_map<uint8_t, RobotRole>>();
  PlannerBase::robot_roles = robot_roles;

  // 設定管理の初期化
  declare_parameter<std::string>("session_config_file_name", "unified_session_config.yaml");
  auto session_config_file_name = get_parameter("session_config_file_name").as_string();
  config_manager_ = std::make_shared<ConfigurationManager>(
    ament_index_cpp::get_package_share_directory("crane_session_controller"),
    session_config_file_name, get_logger());

  // プランナー管理の初期化
  planner_registry_ = std::make_shared<PlannerRegistry>();

  play_situation_sub = create_subscription<crane_msgs::msg::PlaySituation>(
    "/play_situation", 1, [this](const crane_msgs::msg::PlaySituation & msg) {
      play_situation = msg;
      // TODO(HansRobo): 実装
      if (not world_model_ready) {
        return;
      }
      assign(play_situation.command.name);
    });

  timer_process_time_pub = create_publisher<std_msgs::msg::Float32>("~/timer/process_time", 10);
  callback_process_time_pub =
    create_publisher<std_msgs::msg::Float32>("~/callback/process_time", 10);

  using std::chrono::operator""ms;
  timer = rclcpp::create_timer(this, get_clock(), 100ms, [&]() {
    ScopedTimer timer(timer_process_time_pub);
    assign(play_situation.command.name);
  });

  declare_parameter("initial_session", "HALT");
  auto initial_session = get_parameter("initial_session").as_string();
  assign(initial_session);

  world_model->addCallback([this]() {
    ScopedTimer timer(callback_process_time_pub);

    if (not world_model_ready && not world_model->ours().getAvailableRobotIds().empty()) {
      world_model_ready = true;
    }

    // 遅延監視: WorldModel受信完了とSessionController処理開始
    world_model->addDelayCheckpoint("session_controller_start", "callback_triggered");

    crane_msgs::msg::RobotCommands msg;
    msg.header = world_model->getMsg().header;
    msg.on_positive_half = world_model->onPositiveHalf();
    msg.is_yellow = world_model->isYellow();

    // WorldModelのdelay_checkpointsをRobotCommandsにコピー
    msg.delay_checkpoints = world_model->getDelayCheckpoints();

    // ロボットが過不足なく割り当てられているか確認
    auto assigned_robot_ids = getAssignedRobotIds();
    auto observed_robot_ids = world_model->ours().getAvailableRobotIds();
    std::sort(observed_robot_ids.begin(), observed_robot_ids.end());

    bool robot_changed = false;
    if (assigned_robot_ids.size() != observed_robot_ids.size()) {
      RCLCPP_DEBUG_STREAM(
        get_logger(), "ロボットの数が変動しています｜割当数："
                        << assigned_robot_ids.size() << ", 観測数：" << observed_robot_ids.size());
      robot_changed = true;
    } else if (assigned_robot_ids != observed_robot_ids) {
      RCLCPP_DEBUG_STREAM(
        get_logger(), "ロボットの数は変わっていないですが、ラインナップが変動しています\n"
                        << "\tbefore: " << assigned_robot_ids
                        << "\tafter : " << observed_robot_ids);
      robot_changed = true;
    }

    if (robot_changed) {
      assign(play_situation.command.name);
    }

    for (const auto & planner : planner_registry_->getAllPlanners()) {
      auto commands_msg = planner->getRobotCommands();
      ranges::for_each(
        commands_msg.robot_commands, [&](crane_msgs::msg::RobotCommand & robot_command) {
          robot_command.planner_name = planner->name;
        });
      msg.robot_commands.insert(
        msg.robot_commands.end(), commands_msg.robot_commands.begin(),
        commands_msg.robot_commands.end());
      if (planner->getStatus() != PlannerBase::Status::RUNNING) {
        // TODO(HansRobo): プランナが成功・失敗した場合の処理
      }
    }

    // ロボットの優先度を設定(値が高いほど優先度が高い)
    uint8_t robot_priority = 100;
    for (auto & robot_command : msg.robot_commands) {
      robot_command.local_planner_config.priority = --robot_priority;
    }
    msg.header.stamp = now();
    // 遅延監視: SessionController処理完了、RobotCommands送信
    DelayMonitorWrapper::addDelayCheckpoint(
      msg.delay_checkpoints, "session_controller_end", "strategy_computed");

    robot_commands_pub.publish(msg);
    visualizer->flush();
    CraneVisualizerBuffer::publish();

    // 診断情報を更新
    planning_count_++;
    last_planning_time_ = now();
    diagnostic_updater_.force_update();
  });

  session_injection_sub = create_subscription<std_msgs::msg::String>(
    "/session_injection", 1, [this](const std_msgs::msg::String & msg) {
      config_manager_->updateEventMapping("INJECTION", msg.data);
    });

  // 診断Updater設定
  diagnostic_updater_.setHardwareID("session_controller");
  diagnostic_updater_.add(
    "ai_planner/planning_cycle", this, &SessionControllerComponent::updateDiagnostics);
}

auto SessionControllerComponent::assign(const std::string & event_name) -> void
{
  auto session_name_opt = config_manager_->getSessionNameForEvent(event_name);
  if (session_name_opt.has_value()) {
    const std::string & session_name = session_name_opt.value();
    if (session_name != prev_session_name_) {
      RCLCPP_INFO(
        get_logger(),
        "イベント「%s」に対応するセッション「%s」の設定に従ってロボットを割り当てます",
        event_name.c_str(), session_name.c_str());
    }
    prev_session_name_ = session_name;

    try {
      request(session_name, world_model->ours().getAvailableRobotIds());

      // 全セッションの割当状況をログ出力
      logAssignmentIfChanged(buildAssignmentLog());
    } catch (const std::exception & e) {
      std::stringstream what;
      what << "例外が発生しました: \n"
           << e.what() << "\nスタックトレース: \n"
           << boost::stacktrace::stacktrace() << std::endl;
      static int count = 0;
      if (std::ofstream ofs(std::format("/tmp/stacktrace_robot_assign_{}", ++count)); ofs) {
        ofs << what.str() << std::endl;
        ofs.close();
      }
      RCLCPP_ERROR(get_logger(), "%s", what.str().c_str());
    }
  } else {
    RCLCPP_ERROR(
      get_logger(), "イベント「%s」に対応するセッションの設定が見つかりませんでした",
      event_name.c_str());
  }
}

auto SessionControllerComponent::request(
  const std::string & situation, std::vector<uint8_t> selectable_robot_ids) -> void
{
  auto session_capacities_opt = config_manager_->getSessionCapacitiesForSituation(situation);
  if (!session_capacities_opt.has_value()) {
    RCLCPP_ERROR(
      get_logger(),
      "\t「%"
      "s」というSituationに対してロボット割当リクエストが発行されましたが，見つかりませんでした",
      situation.c_str());
    return;
  }

  const auto & session_capacities = session_capacities_opt.value();

  // Pass receiver is only provided when GameAnalysis selects one
  const int pass_receiver = world_model->getMsg().game_analysis.pass_target_id;
  planner_context["AttackerSkill"]["pass_receiver"] =
    (pass_receiver >= 0) ? static_cast<double>(pass_receiver) : -1.0;

  // 前回のプランナーリストを保存し、新しいリストをクリア
  auto prev_available_planners = planner_registry_->getAllPlanners();
  planner_registry_->clear();

  crane_msgs::msg::RobotSelectResults results;

  // 優先順位が高いPlannerから順にロボットを割り当てる
  for (const auto & session_capacity : session_capacities) {
    if (!tryAssignRobotToPlanner(
          session_capacity, selectable_robot_ids, prev_available_planners, results)) {
      // エラーが発生した場合はループを抜ける
      break;
    }
  }

  robot_select_results_pub->publish(results);

  // 割り当てられなかったロボットを待機状態にする
  if (not selectable_robot_ids.empty()) {
    SessionCapacity waiter_session{"waiter", static_cast<int>(selectable_robot_ids.size())};
    tryAssignRobotToPlanner(waiter_session, selectable_robot_ids, prev_available_planners, results);
  }
}

auto SessionControllerComponent::getAssignedRobotIds() const -> std::vector<uint8_t>
{
  auto assigned_robot_ids =
    planner_registry_->getAllPlanners() |
    ranges::views::transform([](const auto & planner) { return planner->getRobots(); }) |
    ranges::views::join | ranges::views::transform([](const auto & robot) { return robot.id; }) |
    ranges::to<std::vector>() | ranges::actions::sort;
  return assigned_robot_ids;
}

auto SessionControllerComponent::buildAssignmentLog() const -> std::string
{
  std::stringstream assignment_log;
  bool first = true;
  for (const auto & planner : planner_registry_->getAllPlanners()) {
    if (!first) {
      assignment_log << ", ";
    }
    first = false;
    assignment_log << planner->name << ":[";
    const auto & robots = planner->getRobots();
    for (size_t i = 0; i < robots.size(); ++i) {
      if (i > 0) {
        assignment_log << ",";
      }
      assignment_log << static_cast<int>(robots[i].id);
    }
    assignment_log << "]";
  }
  return assignment_log.str();
}

auto SessionControllerComponent::logAssignmentIfChanged(const std::string & current_assignment)
  -> void
{
  if (current_assignment != prev_assignment_log_) {
    if (current_assignment.empty()) {
      RCLCPP_INFO(get_logger(), "ロボット割当: なし");
    } else {
      RCLCPP_INFO(get_logger(), "ロボット割当: %s", current_assignment.c_str());
    }
    prev_assignment_log_ = current_assignment;
  }
}

auto SessionControllerComponent::tryAssignRobotToPlanner(
  const SessionCapacity & session_capacity, std::vector<uint8_t> & selectable_robot_ids,
  const std::vector<PlannerBase::SharedPtr> & prev_available_planners,
  crane_msgs::msg::RobotSelectResults & results) -> bool
{
  // 割り当て可能なロボットがない場合はスキップ
  if (session_capacity.selectable_robot_num <= 0 || selectable_robot_ids.empty()) {
    return true;
  }

  crane_msgs::msg::RobotSelectResult result;
  result.name = session_capacity.session_name;
  result.selectable_robots_num = session_capacity.selectable_robot_num;
  std::ranges::copy(selectable_robot_ids, std::back_inserter(result.selectable_robots));

  try {
    // プランナー生成とロボット選択
    auto req = std::make_shared<crane_msgs::srv::RobotSelect::Request>();
    req->selectable_robots_num = session_capacity.selectable_robot_num;
    std::ranges::copy(selectable_robot_ids, std::back_inserter(req->selectable_robots));

    const std::unordered_map<uint8_t, RobotRole> & prev_roles = *PlannerBase::robot_roles;

    // PlannerRegistryを使ってプランナーを取得または生成
    auto planner = planner_registry_->getOrCreatePlanner(
      session_capacity.session_name, world_model, static_cast<rclcpp::Node &>(*this),
      prev_available_planners);

    auto response = planner->doRobotSelect(req, prev_robot_roles_);

    std::ranges::copy(response.selected_robots, std::back_inserter(result.selected_robots));
    results.results.push_back(result);

    // プランナーをレジストリに登録
    planner_registry_->addPlanner(planner);

    if (not selectable_robot_ids.empty()) {
      RCLCPP_DEBUG_STREAM(
        get_logger(), "\tセッション「" << session_capacity.session_name << "」のロボット選択："
                                       << selectable_robot_ids << " -> "
                                       << response.selected_robots);
    }

    // 割当依頼結果の反映
    for (auto selected_robot_id : response.selected_robots) {
      // 割当されたロボットを利用可能ロボットリストから削除
      selectable_robot_ids.erase(
        remove(selectable_robot_ids.begin(), selectable_robot_ids.end(), selected_robot_id),
        selectable_robot_ids.end());
      // 割当されたロボットをロールマップに追加(この情報は他のプランナにも共有される)
      robot_roles->insert_or_assign(
        selected_robot_id, RobotRole{session_capacity.session_name, ""});
    }

    return true;
  } catch (std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "\t「%s」というプランナを呼び出した時に例外が発生しました : %s",
      session_capacity.session_name.c_str(), e.what());
    return false;
  }
}

auto SessionControllerComponent::updateDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat) -> void
{
  // WorldModelが準備できているかチェック
  if (!world_model_ready) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for world model to be ready");
    return;
  }

  // プランニングが実行されているかチェック
  auto time_since_last_planning = (now() - last_planning_time_).seconds();

  if (time_since_last_planning > 1.0) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Planning not running (no update for >1s)");
  } else if (time_since_last_planning > 0.5) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Planning update slow (>0.5s since last update)");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Planning is running normally");
  }

  stat.add("time_since_last_planning", time_since_last_planning);
  stat.add("planning_count", planning_count_);
  stat.add("active_planners", static_cast<int>(planner_registry_->getAllPlanners().size()));
  stat.add("available_robots", static_cast<int>(world_model->ours().getAvailableRobotIds().size()));
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::SessionControllerComponent)
