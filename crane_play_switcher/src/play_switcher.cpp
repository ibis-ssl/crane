// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_play_switcher/play_switcher.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#define CMD_STRING_MAPPING(TYPE, CMD) \
  {                                   \
    TYPE::CMD, #CMD                   \
  }

std::map<int, std::string> raw_command_map = {
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_HALT),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_STOP),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_NORMAL_START),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_FORCE_START),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_PREPARE_KICKOFF_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_PREPARE_KICKOFF_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_PREPARE_PENALTY_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_PREPARE_PENALTY_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_DIRECT_FREE_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_DIRECT_FREE_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_INDIRECT_FREE_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_INDIRECT_FREE_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_TIMEOUT_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_TIMEOUT_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_GOAL_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_GOAL_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_BALL_PLACEMENT_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_BALL_PLACEMENT_BLUE)};

std::map<int, std::string> inplay_command_map = {
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, HALT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, STOP),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_KICKOFF_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_KICKOFF_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_KICKOFF_START),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_KICKOFF_START),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_PENALTY_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_PENALTY_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_PENALTY_START),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_PENALTY_START),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_DIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_DIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_INDIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_INDIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_BALL_PLACEMENT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_BALL_PLACEMENT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, INPLAY)};

namespace crane
{
PlaySwitcher::PlaySwitcher(const rclcpp::NodeOptions & options)
: Node("crane_play_switcher", options)
{
  world_model_ = std::make_unique<WorldModelWrapper>(*this);

  RCLCPP_INFO(get_logger(), "PlaySwitcher is constructed.");
  auto referee_callback = [this](const robocup_ssl_msgs::msg::Referee::SharedPtr msg) -> void {
    this->referee_callback(msg);
  };
  auto world_model_callback = [this](const crane_msgs::msg::WorldModel & msg) -> void {
    this->world_model_callback(msg);
  };

  pub_play_situation_ =
    this->create_publisher<crane_msgs::msg::PlaySituation>("/play_situation", 10);
  sub_decoded_referee_ =
    this->create_subscription<robocup_ssl_msgs::msg::Referee>("/referee", 10, referee_callback);
  sub_world_model_ =
    this->create_subscription<crane_msgs::msg::WorldModel>("world_model", 10, world_model_callback);

  last_command_changed_state_.stamp = now();
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

void PlaySwitcher::referee_callback(const robocup_ssl_msgs::msg::Referee::SharedPtr msg)
{
  using crane_msgs::msg::PlaySituation;
  using robocup_ssl_msgs::msg::Referee;

  static int latest_raw_referee_command = Referee::COMMAND_HALT;

  static struct InplayCommandInfo
  {
    int raw_command;
    int command;
    std::string reason;
  } inplay_command_info;

  inplay_command_info.raw_command = msg->command;

  std::optional<int> next_play_situation = std::nullopt;

  // TODO: robocup_ssl_msgs/msg/Refereeをもう少しわかりやすい形式にする必要あり
  play_situation_msg_.stage = msg->stage;

  if (latest_raw_referee_command != msg->command) {
    //-----------------------------------//
    // NORMAL_START
    //-----------------------------------//

    std::map<int, int> start_command_map;
    NORMAL_START_MAPPING(KICKOFF_PREPARATION, KICKOFF_START);
    NORMAL_START_MAPPING(PENALTY_PREPARATION, PENALTY_START);
    //  start_command_map[PlaySituation::THEIR_KICKOFF_START] = {}

    if (msg->command == Referee::COMMAND_NORMAL_START) {
      next_play_situation = start_command_map[play_situation_msg_.command];
      inplay_command_info.reason =
        "RAWコマンド変化＆NORMAL_START：KICKOFF/PENALTYはPREPARATIONからSTARTに移行";
    }
    //-----------------------------------//
    // FORCE_START
    //-----------------------------------//
    else if (msg->command == Referee::COMMAND_FORCE_START) {
      // FORCE_STARTはインプレイをONにするだけ
      next_play_situation = PlaySituation::INPLAY;
      inplay_command_info.reason = "RAWコマンド変化＆FORCE_START：強制的にINPLAYに突入";
    }
    //-----------------------------------//
    // その他：HALT/STOP/KICKOFF/PENALTY/DIRECT/INDIRECT/PLACEMENT
    //-----------------------------------//
    else {
      // raw command -> crane command
      std::map<int, int> command_map;
      bool is_yellow = msg->yellow.name == "ibis";

      command_map[Referee::COMMAND_HALT] = PlaySituation::HALT;
      command_map[Referee::COMMAND_STOP] = PlaySituation::STOP;

      REDIRECT_MAPPING(TIMEOUT, HALT)
      REDIRECT_MAPPING(GOAL, HALT)

      CMD_MAPPING(is_yellow, PREPARE_KICKOFF, KICKOFF_PREPARATION)
      CMD_MAPPING(is_yellow, PREPARE_PENALTY, PENALTY_PREPARATION)
      CMD_MAPPING(is_yellow, DIRECT_FREE, DIRECT_FREE)
      CMD_MAPPING(is_yellow, INDIRECT_FREE, INDIRECT_FREE)
      CMD_MAPPING(is_yellow, BALL_PLACEMENT, BALL_PLACEMENT)

      next_play_situation = command_map[msg->command];
      inplay_command_info.reason = "RAWコマンド変化：コマンド転送";
    }

  } else {
    //-----------------------------------//
    // INPLAY判定(ルール5.4)
    //-----------------------------------//

    // キックオフ・フリーキック・ペナルティーキック開始後，ボールが少なくとも0.05m動いた
    // 判定は相手番のみ（味方番はその前のNORMAL_STARTで行動開始）
    if (
      play_situation_msg_.command == PlaySituation::THEIR_KICKOFF_START or
      play_situation_msg_.command == PlaySituation::THEIR_DIRECT_FREE or
      play_situation_msg_.command == PlaySituation::THEIR_INDIRECT_FREE or
      play_situation_msg_.command == PlaySituation::THEIR_PENALTY_START) {
      if (0.05 <= (last_command_changed_state_.ball_position - world_model_->ball.pos).norm()) {
        next_play_situation = PlaySituation::INPLAY;
        inplay_command_info.reason = "INPLAY判定：敵ボールが少なくとも0.05m動いた";
      }
    }

    // FORCE START
    // コマンド変化側で実装済み

    // キックオフから10秒経過
    if (
      play_situation_msg_.command == PlaySituation::THEIR_KICKOFF_START &&
      10.0 <= (now() - last_command_changed_state_.stamp).seconds()) {
      next_play_situation = PlaySituation::INPLAY;
      inplay_command_info.reason = "INPLAY判定：敵キックオフから10秒経過";
    }
    // フリーキックからN秒経過（N=5 @DivA, N=10 @DivB）
    if (
      play_situation_msg_.command == PlaySituation::THEIR_DIRECT_FREE or
      play_situation_msg_.command == PlaySituation::THEIR_INDIRECT_FREE) {
      if (5.0 <= (now() - last_command_changed_state_.stamp).seconds()) {
        next_play_situation = PlaySituation::INPLAY;
        inplay_command_info.reason = "INPLAY判定：敵フリーキックからN秒経過（N=5 @DivA, N=10 @DivB)";
      }
    }
  }

  // コマンドが更新されているかを調べる
  if (
    next_play_situation != std::nullopt &&
    next_play_situation.value() != play_situation_msg_.command) {
    play_situation_msg_.command = next_play_situation.value();
    play_situation_msg_.command_updated = true;
    RCLCPP_INFO(get_logger(), "---");
    RCLCPP_INFO(
      get_logger(), "RAW_CMD      : %d (%s)", msg->command, raw_command_map[msg->command].c_str());
    RCLCPP_INFO(
      get_logger(), "INPLAY_CMD   : %d (%s)", play_situation_msg_.command,
      inplay_command_map[play_situation_msg_.command].c_str());
    RCLCPP_INFO(get_logger(), "REASON       : %s", inplay_command_info.reason.c_str());
    RCLCPP_INFO(
      get_logger(), "PREV_CMD_TIME: %f", (now() - last_command_changed_state_.stamp).seconds());

    last_command_changed_state_.stamp = now();
  } else {
    play_situation_msg_.command_updated = false;
  }

  // NOTE: inplay situationはworld_modelのコールバックで更新済み
  pub_play_situation_->publish(play_situation_msg_);

  latest_raw_referee_command = msg->command;
}

void PlaySwitcher::referee_diff_callback() {}

template <typename RobotInfoT>
double calcDistanceFromBall(
  const RobotInfoT & robot_info, const geometry_msgs::msg::Pose2D & ball_pose)
{
  return std::hypot(robot_info.pose.x - ball_pose.x, robot_info.pose.y - ball_pose.y);
}

void PlaySwitcher::world_model_callback(const crane_msgs::msg::WorldModel & msg)
{
  world_model_->update(msg);

  play_situation_msg_.world_model = msg;
  crane_msgs::msg::InPlaySituation inplay_msg;
  geometry_msgs::msg::Pose2D ball_pose = play_situation_msg_.world_model.ball_info.pose;

  // ボールとの距離が最小なロボットid
  // FIXME velocityとaccを利用して，ボールにたどり着くまでの時間でソート
  auto get_ball_closest_robot = [&](const auto & robots) {
    auto ball_pose = play_situation_msg_.world_model.ball_info.pose;
    return std::min_element(robots.begin(), robots.end(), [ball_pose](auto a, auto b) {
      return calcDistanceFromBall(a, ball_pose) < calcDistanceFromBall(b, ball_pose);
    });
  };

  auto nearest_friend = get_ball_closest_robot(play_situation_msg_.world_model.robot_info_ours);
  inplay_msg.nearest_to_ball_robot_id_ours = nearest_friend->id;
  double shortest_distance_ours = calcDistanceFromBall(*nearest_friend, ball_pose);

  // FIXME 所持判定距離閾値を可変にする
  inplay_msg.ball_possession_ours = shortest_distance_ours < 0.1;

  auto nearest_enemy = get_ball_closest_robot(play_situation_msg_.world_model.robot_info_theirs);
  inplay_msg.nearest_to_ball_robot_id_theirs = nearest_enemy->id;
  double shortest_distance_theirs = calcDistanceFromBall(*nearest_enemy, ball_pose);

  // FIXME 所持判定距離閾値を可変にする
  inplay_msg.ball_possession_theirs = shortest_distance_theirs < 0.1;

  play_situation_msg_.inplay_situation = inplay_msg;
}

}  // namespace crane

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crane::PlaySwitcher)
