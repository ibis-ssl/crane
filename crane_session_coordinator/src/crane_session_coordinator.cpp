// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/stacktrace.hpp>
#include <crane_msg_wrappers/delay_monitor_wrapper.hpp>
#include <crane_sessions/session_factory.hpp>
#include <crane_utils/stream.hpp>
#include <crane_utils/time.hpp>
#include <filesystem>
#include <fstream>
#include <range/v3/action/sort.hpp>
#include <range/v3/algorithm/for_each.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>
#include <std_msgs/msg/string.hpp>

#include "crane_session_coordinator/configuration_manager.hpp"
#include "crane_session_coordinator/session_coordinator.hpp"
#include "crane_session_coordinator/session_registry.hpp"

namespace crane
{
SessionCoordinatorComponent::SessionCoordinatorComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("session_controller", options),
  world_model(std::make_shared<WorldModelWrapper>(*this)),
  position_commands_pub(this, "/control_targets", 1, 50., 70.),
  robot_select_results_pub(
    create_publisher<crane_msgs::msg::RobotSelectResults>("/robot_select_results", 10)),
  diagnostic_helper_(
    this, "session_controller", "ai_planner/planning_cycle", this,
    &SessionCoordinatorComponent::updateDiagnostics)
{
  crane::CraneVisualizerBuffer::activate(*this);

  world_model->setBallOwnerCalculatorEnabled(true);

  // 設定管理の初期化
  declare_parameter<std::string>("session_config_file_name", "unified_session_config.yaml");
  auto session_config_file_name = get_parameter("session_config_file_name").as_string();
  config_manager_ = std::make_shared<ConfigurationManager>(
    ament_index_cpp::get_package_share_directory("crane_session_coordinator"),
    session_config_file_name, get_logger());

  // プランナー管理の初期化
  session_registry_ = std::make_shared<SessionRegistry>();

  last_planning_time_ = get_clock()->now();

  // ロボット割当マネージャーの初期化
  robot_allocator_ =
    std::make_unique<RobotAllocator>(config_manager_, session_registry_, get_logger());

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
    if (play_situation.command.name.empty()) {
      return;  // PlaySituation未受信時はスキップ
    }
    assign(play_situation.command.name);
  });

  declare_parameter("initial_session", "HALT");
  initial_session_name = get_parameter("initial_session").as_string();

  world_model->addCallback([this]() { onWorldModelUpdate(); });

  session_injection_sub = create_subscription<std_msgs::msg::String>(
    "/session_injection", 1, [this](const std_msgs::msg::String & msg) {
      config_manager_->updateEventMapping("INJECTION", msg.data);
      // マッピング更新後に即座にassignを実行（タイミング問題を回避）
      assign("INJECTION");
    });

  practice_mode_pub = create_publisher<crane_msgs::msg::PracticeMode>("/practice_mode", 1);

  game_analysis_sub = create_subscription<crane_msgs::msg::GameAnalysis>(
    "/game_analysis", 10, [this](const crane_msgs::msg::GameAnalysis::SharedPtr msg) {
      latest_game_analysis_ = *msg;
      RCLCPP_DEBUG(
        get_logger(), "Received game_analysis: recommended_attacker_id=%d",
        latest_game_analysis_.recommended_attacker_id);
    });
}

auto SessionCoordinatorComponent::assign(const std::string & event_name) -> void
{
  auto session_name_opt = config_manager_->getSessionNameForEvent(event_name);
  if (session_name_opt.has_value()) {
    const std::string & session_name = session_name_opt.value();
    if (session_name != prev_session_name_) {
      RCLCPP_INFO(
        get_logger(),
        "イベント「%s」に対応するセッション「%s」の設定に従ってロボットを割り当てます",
        event_name.c_str(), session_name.c_str());

      // 練習モード設定を publish（situation 切り替え時のみ）
      auto practice_mode = config_manager_->getPracticeModeForSituation(session_name);
      practice_mode_pub->publish(practice_mode);
    }
    prev_session_name_ = session_name;

    try {
      auto results = robot_allocator_->allocate(
        session_name, world_model->ours().robotsWhere().available().getIds(), world_model,
        static_cast<rclcpp::Node &>(*this), play_situation);

      // 結果をパブリッシュ
      robot_select_results_pub->publish(results);

      // 全セッションの割当状況をログ出力
      robot_allocator_->logAssignmentIfChanged(robot_allocator_->buildAssignmentLog());
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

auto SessionCoordinatorComponent::onWorldModelUpdate() -> void
{
  ScopedTimer timer(callback_process_time_pub);

  if (not world_model_ready) {
    world_model_ready = true;

    // 初期セッションを割り当て
    if (!initial_assignment_done && !initial_session_name.empty()) {
      assign(initial_session_name);
      initial_assignment_done = true;
    }
  }

  // 遅延監視: WorldModel受信完了とTacticCoordinator処理開始
  world_model->addDelayCheckpoint("session_controller_start", "callback_triggered");

  // ロボット変動検出と再割当
  auto observed_robot_ids = world_model->ours().robotsWhere().available().getIds();
  if (
    robot_allocator_->detectRobotChange(observed_robot_ids) &&
    !play_situation.command.name.empty()) {
    assign(play_situation.command.name);
  }

  // コマンド収集と構築
  auto msg = collectCommands();

  position_commands_pub.publish(msg);
  visualizer->flush();
  CraneVisualizerBuffer::publish();

  // 診断情報を更新
  planning_count_++;
  last_planning_time_ = now();
  diagnostic_helper_.forceUpdate();
}

auto SessionCoordinatorComponent::updateDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat) -> void
{
  if (!world_model_ready) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for world model to be ready");
    return;
  }

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
  stat.add("active_planners", static_cast<int>(session_registry_->getAllPlanners().size()));
  stat.add(
    "available_robots",
    static_cast<int>(world_model->ours().robotsWhere().available().getIds().size()));
}

auto SessionCoordinatorComponent::collectCommands() -> crane_msgs::msg::RobotCommands
{
  crane_msgs::msg::RobotCommands msg;

  // メタデータを設定
  msg.header = world_model->getMsg().header;
  msg.on_positive_half = world_model->onPositiveHalf();
  msg.is_yellow = world_model->isYellow();
  msg.delay_checkpoints = world_model->getDelayCheckpoints();
  msg.header.stamp = now();
  DelayMonitorWrapper::addDelayCheckpoint(
    msg.delay_checkpoints, "session_controller_end", "strategy_computed");

  // 全プランナーからコマンドを収集
  for (const auto & session : session_registry_->getAllPlanners()) {
    session->setGameAnalysis(latest_game_analysis_);

    auto robot_count = session->getRobots().size();
    auto commands_msg = session->getPositionCommands();
    auto command_count = commands_msg.robot_commands.size();

    if (robot_count > 0 && command_count == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Session「%s」にロボット %zu 台が割り当てられていますが、コマンドが0件生成されました",
        session->name.c_str(), robot_count);
    }

    msg.robot_commands.insert(
      msg.robot_commands.end(), commands_msg.robot_commands.begin(),
      commands_msg.robot_commands.end());
  }

  // ロボット優先度を設定（値が高いほど優先度が高い）
  uint8_t robot_priority = 100;
  for (auto & position_command : msg.robot_commands) {
    position_command.local_planner_config.priority = --robot_priority;
  }

  return msg;
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::SessionCoordinatorComponent)
