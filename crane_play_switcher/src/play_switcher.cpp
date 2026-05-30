// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_play_switcher/play_switcher.hpp"

#include <algorithm>
#include <cmath>
#include <crane_msg_wrappers/play_situation_wrapper.hpp>
#include <crane_utils/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace crane
{
PlaySwitcher::PlaySwitcher(const rclcpp::NodeOptions & options)
: Node("crane_play_switcher", options),
  play_situation_pub(create_publisher<crane_msgs::msg::PlaySituation>("/play_situation", 10))
{
  world_model = std::make_shared<WorldModelWrapper>(*this);

  RCLCPP_INFO(get_logger(), "PlaySwitcher is constructed.");

  process_time_pub = create_publisher<std_msgs::msg::Float32>("~/process_time", 10);

  declare_parameter<std::string>("team_name", "ibis");
  team_name = get_parameter("team_name").as_string();

  decoded_referee_sub = create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 10, [this](const robocup_ssl_msgs::msg::Referee & msg) { referee_callback(msg); });

  last_command_changed_state.stamp = now();
  last_referee_recv_time_ = now();

  constexpr double REFEREE_WATCHDOG_PERIOD_SEC = 0.5;
  referee_watchdog_timer_ = create_wall_timer(
    std::chrono::duration<double>(REFEREE_WATCHDOG_PERIOD_SEC),
    [this]() { check_referee_timeout(); });

  session_injection_sub = create_subscription<std_msgs::msg::String>(
    "/session_injection", 1, [&](const std_msgs::msg::String & msg) {
      // イベント注入（次のレフェリーイベント発生まで有効）
      RCLCPP_INFO_STREAM(this->get_logger(), "[SESSION_INJECTION] " << msg.data);
      play_situation_msg.command =
        getSituationCommandNamedInt(crane_msgs::msg::PlaySituation::INJECTION);
      play_situation_msg.header.stamp = now();
      play_situation_pub->publish(play_situation_msg);
    });
}

#define NORMAL_START_MAPPING(PRE_CMD, CMD)                                        \
  start_command_map[PlaySituation::THEIR_##PRE_CMD] = PlaySituation::THEIR_##CMD; \
  start_command_map[PlaySituation::OUR_##PRE_CMD] = PlaySituation::OUR_##CMD

#define REDIRECT_MAPPING(RAW_CMD, CMD)                                                       \
  command_map[robocup_ssl_msgs::msg::RefereeCommand::RAW_CMD##_YELLOW] = PlaySituation::CMD; \
  command_map[robocup_ssl_msgs::msg::RefereeCommand::RAW_CMD##_BLUE] = PlaySituation::CMD;

#define CMD_MAPPING(is_yellow, RAW_CMD, CMD)                                 \
  if (is_yellow) {                                                           \
    command_map[robocup_ssl_msgs::msg::RefereeCommand::RAW_CMD##_YELLOW] = { \
      PlaySituation::OUR_##CMD};                                             \
    command_map[robocup_ssl_msgs::msg::RefereeCommand::RAW_CMD##_BLUE] = {   \
      PlaySituation::THEIR_##CMD};                                           \
  } else {                                                                   \
    command_map[robocup_ssl_msgs::msg::RefereeCommand::RAW_CMD##_YELLOW] = { \
      PlaySituation::THEIR_##CMD};                                           \
    command_map[robocup_ssl_msgs::msg::RefereeCommand::RAW_CMD##_BLUE] = {   \
      PlaySituation::OUR_##CMD};                                             \
  }

auto PlaySwitcher::referee_callback(const robocup_ssl_msgs::msg::Referee & msg) -> void
{
  ScopedTimer process_timer(process_time_pub);
  last_referee_recv_time_ = now();
  if (referee_timeout_active_) {
    RCLCPP_WARN(get_logger(), "レフェリーメッセージの受信が復帰しました");
    referee_timeout_active_ = false;
    // タイムアウト中に play_situation_msg.command を STOP へ強制遷移させたが
    // latest_raw_referee は据え置いたため、同一 RAW コマンドで復帰すると
    // 後段の差分判定 (latest_raw_referee.command.value != msg.command.value) を
    // すり抜けて STOP に固着してしまう。復帰検出時に latest_raw_referee の
    // コマンドを実在しない sentinel 値へ無効化し、次の差分判定を必ず成立させて
    // 最新の referee コマンドで状況を再評価できるようにする。
    // RefereeCommand.value は int32 で有効値は 0 以上のため、-1 は実在しない
    // sentinel として安全に利用できる。
    constexpr int32_t INVALID_REFEREE_COMMAND = -1;
    latest_raw_referee.command.value = INVALID_REFEREE_COMMAND;
  }
  using crane_msgs::msg::PlaySituation;
  using robocup_ssl_msgs::msg::Referee;

  constexpr double BALL_MOVE_THRESHOLD = 0.05;
  constexpr double KICKOFF_TIMEOUT_SEC = 10.0;
  constexpr double FREE_KICK_TIMEOUT_SEC = 12.0;

  inplay_command_info.raw_command = msg.command.value;
  const bool is_yellow = msg.yellow.name == team_name;

  std::optional<int> next_play_situation = std::nullopt;

  // TODO(HansRobo): robocup_ssl_msgs/msg/Refereeをもう少しわかりやすい形式にする必要あり
  play_situation_msg.stage = getStageNamedInt(msg.stage.value);
  play_situation_msg.command_raw = getRefereeCommandNamedInt(msg.command.value);
  play_situation_msg.next_command_raw.clear();
  if (msg.has_field & msg.NEXT_COMMAND_FIELD_SET) {
    play_situation_msg.next_command_raw.push_back(
      getRefereeCommandNamedInt(msg.next_command.value));
  }
  if (is_yellow) {
    play_situation_msg.our_team_info = msg.yellow;
    play_situation_msg.their_team_info = msg.blue;
  } else {
    play_situation_msg.our_team_info = msg.blue;
    play_situation_msg.their_team_info = msg.yellow;
  }
  play_situation_msg.referee_raw = msg;

  if (
    msg.stage.value == robocup_ssl_msgs::msg::RefereeStage::NORMAL_HALF_TIME or
    msg.stage.value == robocup_ssl_msgs::msg::RefereeStage::EXTRA_HALF_TIME) {
    next_play_situation = PlaySituation::HALF_TIME;
  } else if (msg.stage.value == robocup_ssl_msgs::msg::RefereeStage::POST_GAME) {
    next_play_situation = PlaySituation::POST_GAME;
  } else {
    // 更新があれば判定
    if (latest_raw_referee.command.value != msg.command.value) {
      //-----------------------------------//
      // NORMAL_START
      //-----------------------------------//

      std::map<int, int> start_command_map;
      NORMAL_START_MAPPING(KICKOFF_PREPARATION, KICKOFF_START);
      NORMAL_START_MAPPING(PENALTY_PREPARATION, PENALTY_START);
      //  start_command_map[PlaySituation::THEIR_KICKOFF_START] = {}

      if (msg.command.value == robocup_ssl_msgs::msg::RefereeCommand::NORMAL_START) {
        next_play_situation = start_command_map[play_situation_msg.command.value];
        inplay_command_info.reason =
          "RAWコマンド変化＆NORMAL_START：KICKOFF/"
          "PENALTYはPREPARATIONからSTARTに移行";
      } else if (msg.command.value == robocup_ssl_msgs::msg::RefereeCommand::FORCE_START) {
        //-----------------------------------//
        // FORCE_START
        //-----------------------------------//
        // FORCE_STARTはインプレイをONにするだけ
        next_play_situation = PlaySituation::INPLAY;
        inplay_command_info.reason = "RAWコマンド変化＆FORCE_START：強制的にINPLAYに突入";
      } else if (msg.command.value == robocup_ssl_msgs::msg::RefereeCommand::STOP) {
        //-----------------------------------//
        // STOP
        //-----------------------------------//
        std::map<int, int> stop_command_map = [&]() {
#define NEXT_CMD_MAPPING(is_yellow, NEXT_RAW_CMD, CMD)                            \
  if (is_yellow) {                                                                \
    command_map[robocup_ssl_msgs::msg::RefereeCommand::NEXT_RAW_CMD##_YELLOW] = { \
      PlaySituation::STOP_PRE_OUR_##CMD};                                         \
    command_map[robocup_ssl_msgs::msg::RefereeCommand::NEXT_RAW_CMD##_BLUE] = {   \
      PlaySituation::STOP_PRE_THEIR_##CMD};                                       \
  } else {                                                                        \
    command_map[robocup_ssl_msgs::msg::RefereeCommand::NEXT_RAW_CMD##_YELLOW] = { \
      PlaySituation::STOP_PRE_THEIR_##CMD};                                       \
    command_map[robocup_ssl_msgs::msg::RefereeCommand::NEXT_RAW_CMD##_BLUE] = {   \
      PlaySituation::STOP_PRE_OUR_##CMD};                                         \
  }
          std::map<int, int> command_map;
          NEXT_CMD_MAPPING(is_yellow, PREPARE_PENALTY, PENALTY_PREPARATION);
          NEXT_CMD_MAPPING(is_yellow, PREPARE_KICKOFF, KICKOFF_PREPARATION);
          NEXT_CMD_MAPPING(is_yellow, DIRECT_FREE, DIRECT_FREE);

#undef NEXT_CMD_MAPPING

          command_map[robocup_ssl_msgs::msg::RefereeCommand::FORCE_START] = {
            PlaySituation::STOP_PRE_FORCE_START};

          return command_map;
        }();

        if (
          (msg.has_field & msg.NEXT_COMMAND_FIELD_SET) &&
          stop_command_map.find(msg.next_command.value) != stop_command_map.end()) {
          next_play_situation = stop_command_map.find(msg.next_command.value)->second;
          inplay_command_info.reason = "RAWコマンド変化 & STOP：STOPの場合分け";
        } else {
          next_play_situation = PlaySituation::STOP;
        }
      } else {
        //-----------------------------------//
        // その他：HALT/STOP/KICKOFF/PENALTY/DIRECT/PLACEMENT
        //-----------------------------------//
        // raw command -> crane command
        std::map<int, int> command_map;

        command_map[robocup_ssl_msgs::msg::RefereeCommand::HALT] = PlaySituation::HALT;
        command_map[robocup_ssl_msgs::msg::RefereeCommand::STOP] = PlaySituation::STOP;

        //      REDIRECT_MAPPING(TIMEOUT, HALT)
        REDIRECT_MAPPING(GOAL, HALT)

        CMD_MAPPING(is_yellow, PREPARE_KICKOFF, KICKOFF_PREPARATION)
        CMD_MAPPING(is_yellow, PREPARE_PENALTY, PENALTY_PREPARATION)
        CMD_MAPPING(is_yellow, DIRECT_FREE, DIRECT_FREE)
        CMD_MAPPING(is_yellow, BALL_PLACEMENT, BALL_PLACEMENT)
        CMD_MAPPING(is_yellow, TIMEOUT, TIMEOUT)

        next_play_situation = command_map[msg.command.value];
        inplay_command_info.reason = "RAWコマンド変化：コマンド転送";
      }
    } else {
      if (play_situation_msg.command.value == PlaySituation::INPLAY) {
        // INPLAY 解除
        // if (not world_model->point_checker.isFieldInside(world_model->ball.pos, 0.05)) {
        //   next_play_situation = PlaySituation::STOP;
        //   inplay_command_info.reason = "ボールがフィールド外に出た";
        // }
      } else {
        //-----------------------------------//
        // INPLAY突入判定(ルール5.4)
        //-----------------------------------//

        // キックオフ・フリーキック・ペナルティーキック開始後，ボールが少なくとも0.05m動いた
        if (
          play_situation_msg.command.value == PlaySituation::THEIR_KICKOFF_START or
          play_situation_msg.command.value == PlaySituation::THEIR_DIRECT_FREE or
          // 敵PKのINPLAYはOUR_PENALTY_STARTとして実装しているのでINPLAY遷移はしない
          // play_situation_msg.command.value == PlaySituation::THEIR_PENALTY_START or
          play_situation_msg.command.value == PlaySituation::OUR_KICKOFF_START or
          play_situation_msg.command.value == PlaySituation::OUR_DIRECT_FREE
          // 味方PKのINPLAYはOUR_PENALTY_STARTとして実装しているのでINPLAY遷移はしない
          // play_situation_msg.command.value == PlaySituation::OUR_PENALTY_START
        ) {
          const double ball_movement =
            (last_command_changed_state.ball_position - world_model->ball().pos).norm();
          if (BALL_MOVE_THRESHOLD <= ball_movement) {
            next_play_situation = PlaySituation::INPLAY;
            inplay_command_info.reason = "INPLAY判定：敵ボールが少なくとも0.05m動いた(移動量: " +
                                         std::to_string(ball_movement) + "m)";
          }
        }

        // FORCE START
        // コマンド変化側で実装済み

        // キックオフから10秒経過
        if (
          play_situation_msg.command.value == PlaySituation::THEIR_KICKOFF_START &&
          KICKOFF_TIMEOUT_SEC <= (now() - last_command_changed_state.stamp).seconds()) {
          next_play_situation = PlaySituation::INPLAY;
          inplay_command_info.reason = "INPLAY判定：敵キックオフから10秒経過";
        }
        // フリーキックからN秒経過（N=5 @DivA, N=10 @DivB）
        if (play_situation_msg.command.value == PlaySituation::THEIR_DIRECT_FREE) {
          if (FREE_KICK_TIMEOUT_SEC <= (now() - last_command_changed_state.stamp).seconds()) {
            next_play_situation = PlaySituation::INPLAY;
            inplay_command_info.reason = "INPLAY判定：敵フリーキックから12秒経過";
          }
        }
      }
    }
  }

  // コマンドが更新されているかを調べる
  if (
    next_play_situation != std::nullopt &&
    next_play_situation.value() != static_cast<int>(play_situation_msg.command.value)) {
    play_situation_msg.command = getSituationCommandNamedInt(next_play_situation.value());
    play_situation_msg.reason_text = inplay_command_info.reason;

    RCLCPP_DEBUG_STREAM(
      get_logger(), "---\n"
                      << "RAW_CMD      : " << msg.command.value << " ("
                      << getRefereeCommandText(msg.command.value) << ")\n"
                      << "INPLAY_CMD   : " << play_situation_msg.command.value << " ("
                      << play_situation_msg.command.name << ")\n"
                      << "REASON       : " << inplay_command_info.reason << "\n"
                      << "PREV_CMD_TIME: " << (now() - last_command_changed_state.stamp).seconds());

    last_command_changed_state.stamp = now();
    last_command_changed_state.ball_position = world_model->ball().pos;

    if (msg.has_field & msg.DESIGNATED_POSITION_FIELD_SET) {
      play_situation_msg.placement_position.x = msg.designated_position.x;
      play_situation_msg.placement_position.y = msg.designated_position.y;
    }

    // パブリッシュはコマンド更新時のみ
    play_situation_msg.header.stamp = now();
    play_situation_pub->publish(play_situation_msg);
  }

  latest_raw_referee = msg;
}

#undef NORMAL_START_MAPPING
#undef REDIRECT_MAPPING
#undef CMD_MAPPING

void PlaySwitcher::check_referee_timeout()
{
  using crane_msgs::msg::PlaySituation;
  constexpr double REFEREE_TIMEOUT_SEC = 1.0;

  const rclcpp::Time current_time = now();
  const double elapsed_sec = (current_time - last_referee_recv_time_).seconds();

  if (elapsed_sec <= REFEREE_TIMEOUT_SEC) {
    return;
  }

  const int current_cmd = play_situation_msg.command.value;
  if (
    current_cmd == PlaySituation::HALT || current_cmd == PlaySituation::STOP ||
    current_cmd == PlaySituation::HALF_TIME || current_cmd == PlaySituation::POST_GAME) {
    return;
  }

  if (referee_timeout_active_) {
    return;
  }

  RCLCPP_WARN(
    get_logger(), "レフェリーメッセージが%.1f秒受信できていません。安全のためSTOPに遷移します。",
    elapsed_sec);
  referee_timeout_active_ = true;

  play_situation_msg.command = getSituationCommandNamedInt(PlaySituation::STOP);
  play_situation_msg.reason_text = "レフェリーメッセージタイムアウト：安全のためSTOPに遷移";
  play_situation_msg.header.stamp = current_time;
  play_situation_pub->publish(play_situation_msg);
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::PlaySwitcher)
