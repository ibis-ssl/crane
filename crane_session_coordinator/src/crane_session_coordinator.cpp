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

  // 診断レポーターの初期化
  diagnostics_reporter_ =
    std::make_unique<DiagnosticsReporter>(get_clock(), session_registry_, get_logger());

  // コマンドアグリゲーターの初期化
  command_aggregator_ = std::make_unique<CommandAggregator>(session_registry_);

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
    });

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
    }
    prev_session_name_ = session_name;

    try {
      auto results = robot_allocator_->allocate(
        session_name, world_model->ours().robotsWhere().available().getIds(), world_model,
        static_cast<rclcpp::Node &>(*this));

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
  auto msg = command_aggregator_->collectCommands(world_model, now(), latest_game_analysis_);

  position_commands_pub.publish(msg);
  visualizer->flush();
  CraneVisualizerBuffer::publish();

  // 診断情報を更新
  diagnostics_reporter_->recordCycle();
  diagnostic_helper_.forceUpdate();
}

auto SessionCoordinatorComponent::updateDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat) -> void
{
  diagnostics_reporter_->updateDiagnostics(stat, world_model_ready, world_model);
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(crane::SessionCoordinatorComponent)
