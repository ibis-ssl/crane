// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_play_switcher/play_switcher.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace crane
{
PlaySwitcher::PlaySwitcher(const rclcpp::NodeOptions & options)
: Node("crane_play_switcher", options)
{
  RCLCPP_INFO(this->get_logger(), "PlaySwitcher is constructed.");
  auto referee_callback = [this](const robocup_ssl_msgs::msg::Referee::SharedPtr msg) -> void {
    this->referee_callback(msg);
  };
  auto world_model_callback = [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void {
    this->world_model_callback(msg);
  };
  // FIXME トピック名を合わせる
  pub_play_situation_ =
    this->create_publisher<crane_msgs::msg::PlaySituation>("/play_situation", 10);
  sub_decoded_referee_ =
    this->create_subscription<robocup_ssl_msgs::msg::Referee>("/referee", 10, referee_callback);
  sub_world_model_ =
    this->create_subscription<crane_msgs::msg::WorldModel>("world_model", 10, world_model_callback);
}

#define REDIRECT_MAPPING(RAW_CMD, CMD)                                           \
  command_map[Referee::COMMAND_##RAW_CMD##_YELLOW] = {PlaySituation::CMD, #CMD}; \
  command_map[Referee::COMMAND_##RAW_CMD##_BLUE] = {PlaySituation::CMD, #CMD};

#define CMD_MAPPING(is_yellow, RAW_CMD, CMD)                     \
  if (is_yellow) {                                               \
    command_map[Referee::COMMAND_##RAW_CMD##_YELLOW] = {         \
      PlaySituation::OUR_##CMD, std::string("OUR_") + #CMD};     \
    command_map[Referee::COMMAND_##RAW_CMD##_BLUE] = {           \
      PlaySituation::THEIR_##CMD, std::string("THEIR_") + #CMD}; \
  } else {                                                       \
    command_map[Referee::COMMAND_##RAW_CMD##_YELLOW] = {         \
      PlaySituation::THEIR_##CMD, std::string("THEIR_") + #CMD}; \
    command_map[Referee::COMMAND_##RAW_CMD##_BLUE] = {           \
      PlaySituation::OUR_##CMD, std::string("OUR_") + #CMD};     \
  }

void PlaySwitcher::referee_callback(const robocup_ssl_msgs::msg::Referee::SharedPtr msg)
{
  bool is_difference = false;
  auto previous_play_situation = play_situation_msg_;
  std::string command_str;

  using crane_msgs::msg::PlaySituation;
  using robocup_ssl_msgs::msg::Referee;
  // TODO: robocup_ssl_msgs/msg/Refereeをもう少しわかりやすい形式にする必要あり
  play_situation_msg_.stage = msg->stage;

  // raw command -> crane command
  std::map<int, std::pair<int, std::string>> start_command_map;
  start_command_map[PlaySituation::OUR_KICKOFF_PREPARATION] = {
    PlaySituation::OUR_KICKOFF_START, "OUR_KICKOFF_START"};
  start_command_map[PlaySituation::THEIR_KICKOFF_PREPARATION] = {
    PlaySituation::THEIR_KICKOFF_START, "THEIR_KICKOFF_START"};
  start_command_map[PlaySituation::OUR_PENALTY_PREPARATION] = {
    PlaySituation::OUR_PENALTY_START, "OUR_PENALTY_START"};
  start_command_map[PlaySituation::THEIR_PENALTY_PREPARATION] = {
    PlaySituation::THEIR_PENALTY_START, "THEIR_PENALTY_START"};

  // NORMAL_STARTの意味を解釈
  if (msg->command == Referee::COMMAND_NORMAL_START) {
    //    RCLCPP_INFO(get_logger(), "NORMAL_START: %s", start_command_map[play_situation_msg_.command].second);
    play_situation_msg_.command = start_command_map[play_situation_msg_.command].first;
    command_str = start_command_map[play_situation_msg_.command].second;
  } else if (msg->command == Referee::COMMAND_FORCE_START) {
    // FORCE_STARTはインプレイをONにするだけ
    play_situation_msg_.command = PlaySituation::INPLAY;
  } else {
    // raw command -> crane command
    std::map<int, std::pair<int, std::string>> command_map;
    bool is_yellow = msg->yellow.name == "ibis";

    command_map[Referee::COMMAND_HALT] = {PlaySituation::HALT, "HALT"};
    command_map[Referee::COMMAND_STOP] = {PlaySituation::STOP, "STOP"};
    REDIRECT_MAPPING(TIMEOUT, HALT)
    REDIRECT_MAPPING(GOAL, STOP)

    CMD_MAPPING(is_yellow, PREPARE_KICKOFF, KICKOFF_PREPARATION)
    CMD_MAPPING(is_yellow, PREPARE_PENALTY, PENALTY_PREPARATION)
    CMD_MAPPING(is_yellow, DIRECT_FREE, DIRECT_FREE)
    CMD_MAPPING(is_yellow, INDIRECT_FREE, INDIRECT_FREE)
    CMD_MAPPING(is_yellow, BALL_PLACEMENT, BALL_PLACEMENT)

    play_situation_msg_.command = command_map[msg->command].first;
    command_str = command_map[msg->command].second;
  }

  // コマンドが更新されているかを調べる
  if (play_situation_msg_.command != previous_play_situation.command) {
    is_difference = true;
  }

  if (is_difference) {
    referee_diff_callback();
    RCLCPP_INFO(this->get_logger(), command_str.c_str());
  }

  // NOTE: inplay situationはworld_modelのコールバックで更新済み
  // play_situation_msg_.placement_position = msg->placement_position;
  pub_play_situation_->publish(play_situation_msg_);
}

void PlaySwitcher::referee_diff_callback() { RCLCPP_INFO(get_logger(), "コマンド変化！"); }

template <typename RobotInfoT>
double calcDistanceFromBall(
  const RobotInfoT & robot_info, const geometry_msgs::msg::Pose2D & ball_pose)
{
  return std::hypot(robot_info.pose.x - ball_pose.x, robot_info.pose.y - ball_pose.y);
}

void PlaySwitcher::world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg)
{
  play_situation_msg_.world_model = *msg;
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crane::PlaySwitcher)
