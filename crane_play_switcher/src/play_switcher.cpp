// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_play_switcher/play_switcher.hpp"

#include <algorithm>
#include <cmath>
#include <crane_basics/time.hpp>
#include <crane_msg_wrappers/play_situation_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace crane
{
PlaySwitcher::PlaySwitcher(const rclcpp::NodeOptions & options)
: Node("crane_play_switcher", options)
{
  world_model = std::make_shared<WorldModelWrapper>(*this);

  RCLCPP_INFO(get_logger(), "PlaySwitcher is constructed.");

  play_situation_pub = create_publisher<crane_msgs::msg::PlaySituation>("/play_situation", 10);

  process_time_pub = create_publisher<std_msgs::msg::Float32>("~/process_time", 10);

  declare_parameter<std::string>("team_name", "ibis");
  team_name = get_parameter("team_name").as_string();

  decoded_referee_sub = create_subscription<robocup_ssl_msgs::msg::Referee>(
    "/referee", 10, [this](const robocup_ssl_msgs::msg::Referee & msg) { referee_callback(msg); });

  last_command_changed_state.stamp = now();
}

#define NORMAL_START_MAPPING(PRE_CMD, CMD)                                        \
  start_command_map[PlaySituation::THEIR_##PRE_CMD] = PlaySituation::THEIR_##CMD; \
  start_command_map[PlaySituation::OUR_##PRE_CMD] = PlaySituation::OUR_##CMD

#define REDIRECT_MAPPING(RAW_CMD, CMD)                                   \
  command_map[Referee::COMMAND_##RAW_CMD##_YELLOW] = PlaySituation::CMD; \
  command_map[Referee::COMMAND_##RAW_CMD##_BLUE] = PlaySituation::CMD;

#define CMD_MAPPING(is_yellow, RAW_CMD, CMD)                                         \
  if (is_yellow) {                                                                   \
    command_map[Referee::COMMAND_##RAW_CMD##_YELLOW] = {PlaySituation::OUR_##CMD};   \
    command_map[Referee::COMMAND_##RAW_CMD##_BLUE] = {PlaySituation::THEIR_##CMD};   \
  } else {                                                                           \
    command_map[Referee::COMMAND_##RAW_CMD##_YELLOW] = {PlaySituation::THEIR_##CMD}; \
    command_map[Referee::COMMAND_##RAW_CMD##_BLUE] = {PlaySituation::OUR_##CMD};     \
  }

void PlaySwitcher::referee_callback(const robocup_ssl_msgs::msg::Referee & msg)
{
  ScopedTimer process_timer(process_time_pub);
  using crane_msgs::msg::PlaySituation;
  using robocup_ssl_msgs::msg::Referee;

  static int latest_raw_referee_command = Referee::COMMAND_HALT;

  static struct InplayCommandInfo
  {
    int raw_command;
    int command;
    std::string reason;
  } inplay_command_info;

  inplay_command_info.raw_command = msg.command;

  std::optional<int> next_play_situation = std::nullopt;

  // TODO(HansRobo): robocup_ssl_msgs/msg/Refereeをもう少しわかりやすい形式にする必要あり
  play_situation_msg.stage = msg.stage;

  if (latest_raw_referee_command != static_cast<int>(msg.command)) {
    //-----------------------------------//
    // NORMAL_START
    //-----------------------------------//

    std::map<int, int> start_command_map;
    NORMAL_START_MAPPING(KICKOFF_PREPARATION, KICKOFF_START);
    NORMAL_START_MAPPING(PENALTY_PREPARATION, PENALTY_START);
    //  start_command_map[PlaySituation::THEIR_KICKOFF_START] = {}

    if (msg.command == Referee::COMMAND_NORMAL_START) {
      next_play_situation = start_command_map[play_situation_msg.command];
      inplay_command_info.reason =
        "RAWコマンド変化＆NORMAL_START：KICKOFF/"
        "PENALTYはPREPARATIONからSTARTに移行";
    } else if (msg.command == Referee::COMMAND_FORCE_START) {
      //-----------------------------------//
      // FORCE_START
      //-----------------------------------//
      // FORCE_STARTはインプレイをONにするだけ
      next_play_situation = PlaySituation::INPLAY;
      inplay_command_info.reason = "RAWコマンド変化＆FORCE_START：強制的にINPLAYに突入";
    } else {
      //-----------------------------------//
      // その他：HALT/STOP/KICKOFF/PENALTY/DIRECT/INDIRECT/PLACEMENT
      //-----------------------------------//
      // raw command -> crane command
      std::map<int, int> command_map;
      bool is_yellow = msg.yellow.name == team_name;

      command_map[Referee::COMMAND_HALT] = PlaySituation::HALT;
      command_map[Referee::COMMAND_STOP] = PlaySituation::STOP;

      REDIRECT_MAPPING(TIMEOUT, HALT)
      REDIRECT_MAPPING(GOAL, HALT)

      CMD_MAPPING(is_yellow, PREPARE_KICKOFF, KICKOFF_PREPARATION)
      CMD_MAPPING(is_yellow, PREPARE_PENALTY, PENALTY_PREPARATION)
      CMD_MAPPING(is_yellow, DIRECT_FREE, DIRECT_FREE)
      CMD_MAPPING(is_yellow, INDIRECT_FREE, INDIRECT_FREE)
      CMD_MAPPING(is_yellow, BALL_PLACEMENT, BALL_PLACEMENT)

      next_play_situation = command_map[msg.command];
      inplay_command_info.reason = "RAWコマンド変化：コマンド転送";
    }

  } else {
    //-----------------------------------//
    // INPLAY突入判定(ルール5.4)
    //-----------------------------------//

    // キックオフ・フリーキック・ペナルティーキック開始後，ボールが少なくとも0.05m動いた
    if (
      play_situation_msg.command == PlaySituation::THEIR_KICKOFF_START or
      play_situation_msg.command == PlaySituation::THEIR_DIRECT_FREE or
      play_situation_msg.command == PlaySituation::THEIR_INDIRECT_FREE or
      // 敵PKのINPLAYはOUR_PENALTY_STARTとして実装しているのでINPLAY遷移はしない
      // play_situation_msg.command == PlaySituation::THEIR_PENALTY_START or
      play_situation_msg.command == PlaySituation::OUR_KICKOFF_START or
      play_situation_msg.command == PlaySituation::OUR_DIRECT_FREE or
      // 味方PKのINPLAYはOUR_PENALTY_STARTとして実装しているのでINPLAY遷移はしない
      // play_situation_msg.command == PlaySituation::OUR_PENALTY_START or
      play_situation_msg.command == PlaySituation::OUR_INDIRECT_FREE) {
      if (0.05 <= (last_command_changed_state.ball_position - world_model->ball.pos).norm()) {
        next_play_situation = PlaySituation::INPLAY;
        inplay_command_info.reason =
          "INPLAY判定：敵ボールが少なくとも0.05m動いた(移動量: " +
          std::to_string(
            (last_command_changed_state.ball_position - world_model->ball.pos).norm()) +
          "m)";
      }
    }

    // FORCE START
    // コマンド変化側で実装済み

    // キックオフから10秒経過
    if (
      play_situation_msg.command == PlaySituation::THEIR_KICKOFF_START &&
      10.0 <= (now() - last_command_changed_state.stamp).seconds()) {
      next_play_situation = PlaySituation::INPLAY;
      inplay_command_info.reason = "INPLAY判定：敵キックオフから10秒経過";
    }
    // フリーキックからN秒経過（N=5 @DivA, N=10 @DivB）
    if (
      play_situation_msg.command == PlaySituation::THEIR_DIRECT_FREE or
      play_situation_msg.command == PlaySituation::THEIR_INDIRECT_FREE) {
      if (30.0 <= (now() - last_command_changed_state.stamp).seconds()) {
        next_play_situation = PlaySituation::INPLAY;
        inplay_command_info.reason =
          "INPLAY判定：敵フリーキックからN秒経過（N=5 @DivA, N=10 @DivB)";
      }
    }
  }

  // コマンドが更新されているかを調べる
  if (
    next_play_situation != std::nullopt &&
    next_play_situation.value() != static_cast<int>(play_situation_msg.command)) {
    play_situation_msg.command = next_play_situation.value();
    play_situation_msg.reason_text = inplay_command_info.reason;

    RCLCPP_INFO(get_logger(), "---");
    RCLCPP_INFO(
      get_logger(), "RAW_CMD      : %d (%s)", msg.command,
      PlaySituationWrapper::getRefereeCommandText(msg.command).c_str());
    RCLCPP_INFO(
      get_logger(), "INPLAY_CMD   : %d (%s)", play_situation_msg.command,
      PlaySituationWrapper::getSituationCommandText(play_situation_msg.command).c_str());
    RCLCPP_INFO(get_logger(), "REASON       : %s", inplay_command_info.reason.c_str());
    RCLCPP_INFO(
      get_logger(), "PREV_CMD_TIME: %f", (now() - last_command_changed_state.stamp).seconds());

    last_command_changed_state.stamp = now();
    last_command_changed_state.ball_position = world_model->ball.pos;

    if (msg.designated_position.size() > 0) {
      play_situation_msg.placement_position.x = msg.designated_position[0].x;
      play_situation_msg.placement_position.y = msg.designated_position[0].y;
    }

    // パブリッシュはコマンド更新時のみ
    play_situation_msg.header.stamp = now();
    play_situation_pub->publish(play_situation_msg);
  }

  latest_raw_referee_command = msg.command;
}

template <typename RobotInfoT>
double calcDistanceFromBall(
  const RobotInfoT & robot_info, const geometry_msgs::msg::Pose2D & ball_pose)
{
  return std::hypot(robot_info.pose.x - ball_pose.x, robot_info.pose.y - ball_pose.y);
}
}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::PlaySwitcher)
