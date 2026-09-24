// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLAY_SWITCHER__REFEREE_COMMAND_TRANSITION_HPP_
#define CRANE_PLAY_SWITCHER__REFEREE_COMMAND_TRANSITION_HPP_

#include <crane_msgs/msg/play_situation.hpp>
#include <map>
#include <optional>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <string>

namespace crane
{
struct CommandTransition
{
  int next_play_situation;
  std::string reason;
  // タイムアウト前の状態をそのまま戻した。キックオフ・フリーキックの経過時間とボール移動量の
  // 基準はタイムアウト前のものを使い続ける
  bool restored_before_timeout = false;
};

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

// RAW コマンドから次の PlaySituation を決める。current_play_situation は NORMAL_START で
// どの開始状態へ進むかの判定にだけ使う
inline auto decideRawCommandTransition(
  int current_play_situation, const robocup_ssl_msgs::msg::Referee & msg, bool is_yellow)
  -> CommandTransition
{
  using crane_msgs::msg::PlaySituation;
  using robocup_ssl_msgs::msg::RefereeCommand;

  if (msg.command.value == RefereeCommand::NORMAL_START) {
    std::map<int, int> start_command_map;
    NORMAL_START_MAPPING(KICKOFF_PREPARATION, KICKOFF_START);
    NORMAL_START_MAPPING(PENALTY_PREPARATION, PENALTY_START);
    // PREPARATION 以外からの NORMAL_START は HALT に倒す
    const auto it = start_command_map.find(current_play_situation);
    return {
      it != start_command_map.end() ? it->second : PlaySituation::HALT,
      "RAWコマンド変化＆NORMAL_START：KICKOFF/PENALTYはPREPARATIONからSTARTに移行"};
  }
  if (msg.command.value == RefereeCommand::FORCE_START) {
    // FORCE_STARTはインプレイをONにするだけ
    return {PlaySituation::INPLAY, "RAWコマンド変化＆FORCE_START：強制的にINPLAYに突入"};
  }
  if (msg.command.value == RefereeCommand::STOP) {
    std::map<int, int> command_map;
    NEXT_CMD_MAPPING(is_yellow, PREPARE_PENALTY, PENALTY_PREPARATION);
    NEXT_CMD_MAPPING(is_yellow, PREPARE_KICKOFF, KICKOFF_PREPARATION);
    NEXT_CMD_MAPPING(is_yellow, DIRECT_FREE, DIRECT_FREE);
    command_map[RefereeCommand::FORCE_START] = {PlaySituation::STOP_PRE_FORCE_START};

    if (msg.has_field & msg.NEXT_COMMAND_FIELD_SET) {
      if (const auto it = command_map.find(msg.next_command.value); it != command_map.end()) {
        return {it->second, "RAWコマンド変化 & STOP：STOPの場合分け"};
      }
    }
    return {PlaySituation::STOP, "RAWコマンド変化 & STOP"};
  }

  std::map<int, int> command_map;
  command_map[RefereeCommand::HALT] = PlaySituation::HALT;
  command_map[RefereeCommand::STOP] = PlaySituation::STOP;

  REDIRECT_MAPPING(GOAL, HALT)

  CMD_MAPPING(is_yellow, PREPARE_KICKOFF, KICKOFF_PREPARATION)
  CMD_MAPPING(is_yellow, PREPARE_PENALTY, PENALTY_PREPARATION)
  CMD_MAPPING(is_yellow, DIRECT_FREE, DIRECT_FREE)
  CMD_MAPPING(is_yellow, BALL_PLACEMENT, BALL_PLACEMENT)
  CMD_MAPPING(is_yellow, TIMEOUT, TIMEOUT)

  return {command_map[msg.command.value], "RAWコマンド変化：コマンド転送"};
}

#undef NORMAL_START_MAPPING
#undef REDIRECT_MAPPING
#undef CMD_MAPPING
#undef NEXT_CMD_MAPPING

// RAW コマンドが変わったとき、またはタイムアウトから復帰したときの遷移を決める。
// どちらでもなければ std::nullopt（INPLAY 突入判定は呼び出し側で行う）。
// play_situation_before_timeout はタイムアウトからの復帰時だけ値を持つ。
inline auto decideCommandTransition(
  int current_play_situation, const robocup_ssl_msgs::msg::Referee & latest_raw,
  const robocup_ssl_msgs::msg::Referee & msg, bool is_yellow,
  std::optional<int> play_situation_before_timeout) -> std::optional<CommandTransition>
{
  if (play_situation_before_timeout) {
    // ssl-game-controller は新しいコマンドを出すたびに command_counter を増やし、定期再送では
    // 同じ値を送る。コマンドも counter も同じなら、途切れている間に新しいコマンドは出ていない
    if (
      latest_raw.command.value == msg.command.value &&
      latest_raw.command_counter == msg.command_counter) {
      return CommandTransition{
        *play_situation_before_timeout, "レフェリー復帰：タイムアウト前の状態に戻す", true};
    }
    return decideRawCommandTransition(*play_situation_before_timeout, msg, is_yellow);
  }
  if (latest_raw.command.value != msg.command.value) {
    return decideRawCommandTransition(current_play_situation, msg, is_yellow);
  }
  return std::nullopt;
}
}  // namespace crane

#endif  // CRANE_PLAY_SWITCHER__REFEREE_COMMAND_TRANSITION_HPP_
