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

#define REDIRECT_MAPPING(RAW_CMD, CMD)                                   \
  command_map[Referee::COMMAND_##RAW_CMD##_YELLOW] = PlaySituation::CMD; \
  command_map[Referee::COMMAND_##RAW_CMD##_BLUE] = PlaySituation::CMD;

#define CMD_MAPPING(is_yellow, RAW_CMD, CMD)                                       \
  if (is_yellow) {                                                                 \
    command_map[Referee::COMMAND_##RAW_CMD##_YELLOW] = PlaySituation::OUR_##CMD;   \
    command_map[Referee::COMMAND_##RAW_CMD##_BLUE] = PlaySituation::THEIR_##CMD;   \
  } else {                                                                         \
    command_map[Referee::COMMAND_##RAW_CMD##_YELLOW] = PlaySituation::THEIR_##CMD; \
    command_map[Referee::COMMAND_##RAW_CMD##_BLUE] = PlaySituation::OUR_##CMD;     \
  }

void PlaySwitcher::referee_callback(const robocup_ssl_msgs::msg::Referee::SharedPtr msg)
{
  using crane_msgs::msg::PlaySituation;
  using robocup_ssl_msgs::msg::Referee;
  // TODO: robocup_ssl_msgs/msg/Refereeをもう少しわかりやすい形式にする必要あり
  play_situation_msg_.stage = msg->stage;

  // raw command -> crane command
  std::map<int, int> start_command_map;
  start_command_map[PlaySituation::OUR_KICKOFF_PREPARATION] = PlaySituation::OUR_KICKOFF_START;
  start_command_map[PlaySituation::THEIR_KICKOFF_PREPARATION] = PlaySituation::THEIR_KICKOFF_START;
  start_command_map[PlaySituation::OUR_PENALTY_PREPARATION] = PlaySituation::OUR_PENALTY_START;
  start_command_map[PlaySituation::THEIR_PENALTY_PREPARATION] = PlaySituation::THEIR_PENALTY_START;

  // NORMAL_STARTの意味を解釈
  if (msg->command == Referee::COMMAND_NORMAL_START) {
    play_situation_msg_.command = start_command_map[play_situation_msg_.command];
  } else if (msg->command == Referee::COMMAND_FORCE_START) {
    // FORCE_STARTはインプレイをONにするだけ
    play_situation_msg_.is_inplay = PlaySituation::INPLAY;
  } else {
    // raw command -> crane command
    std::map<int, int> command_map;
    bool is_yellow = msg->yellow.name == "ibis";

    command_map[Referee::COMMAND_HALT] = PlaySituation::HALT;
    command_map[Referee::COMMAND_STOP] = PlaySituation::STOP;
    REDIRECT_MAPPING(TIMEOUT, HALT)
    REDIRECT_MAPPING(GOAL, STOP)

    CMD_MAPPING(is_yellow, PREPARE_KICKOFF, KICKOFF_PREPARATION)
    CMD_MAPPING(is_yellow, PREPARE_PENALTY, PENALTY_PREPARATION)
    CMD_MAPPING(is_yellow, DIRECT_FREE, DIRECT_FREE)
    CMD_MAPPING(is_yellow, INDIRECT_FREE, INDIRECT_FREE)
    CMD_MAPPING(is_yellow, BALL_PLACEMENT, BALL_PLACEMENT)

    play_situation_msg_.command = start_command_map[msg->command];
  }

  //  TODO: inplay situationを埋める
  // play_situation_msg_.placement_position = msg->placement_position;
  pub_play_situation_->publish(play_situation_msg_);
}

template <typename RobotInfoT>
double calcDistanceFromBall(
  const RobotInfoT & robot_info, const geometry_msgs::msg::Pose2D & ball_pose)
{
  return std::hypot(robot_info.pose.x - ball_pose.x, robot_info.pose.y - ball_pose.y);
}

void PlaySwitcher::world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg)
{
  play_situation_msg_.world_model = *msg;
  if (play_situation_msg_.is_inplay) {
    crane_msgs::msg::InPlaySituation tmp_msg;
    geometry_msgs::msg::Pose2D ball_pose = play_situation_msg_.world_model.ball_info.pose;
    // geometry_msgs::msg::Pose2D ball_velocity
    //   = play_situation_msg_.world_model.ball_info.velocity;

    // ボールとの距離が最小なロボットid
    // FIXME velocityとaccを利用して，ボールにたどり着くまでの時間でソート
    const std::vector<crane_msgs::msg::RobotInfoOurs> & ours =
      play_situation_msg_.world_model.robot_info_ours;
    auto nearest_ours_itr = std::min_element(
      ours.begin(), ours.end(),
      [ball_pose](crane_msgs::msg::RobotInfoOurs a, crane_msgs::msg::RobotInfoOurs b) {
        return calcDistanceFromBall(a, ball_pose) < calcDistanceFromBall(b, ball_pose);
      });
    tmp_msg.nearest_to_ball_robot_id_ours = nearest_ours_itr->id;
    double shortest_distance_ours = calcDistanceFromBall(*nearest_ours_itr, ball_pose);

    // FIXME 所持判定距離閾値を可変にする
    tmp_msg.ball_possession_ours = shortest_distance_ours < 1.0e-3;

    // ここから上のoursのやつのコピペしてtheirsに変更
    const std::vector<crane_msgs::msg::RobotInfoTheirs> & theirs =
      play_situation_msg_.world_model.robot_info_theirs;
    auto nearest_theirs_itr = std::min_element(
      theirs.begin(), theirs.end(),
      [ball_pose](crane_msgs::msg::RobotInfoTheirs a, crane_msgs::msg::RobotInfoTheirs b) {
        return calcDistanceFromBall(a, ball_pose) < calcDistanceFromBall(b, ball_pose);
      });
    tmp_msg.nearest_to_ball_robot_id_theirs = nearest_theirs_itr->id;
    double shortest_distance_theirs = calcDistanceFromBall(*nearest_theirs_itr, ball_pose);

    // FIXME 所持判定距離閾値を可変にする
    tmp_msg.ball_possession_theirs = shortest_distance_theirs < 1.0e-3;
    // ここまで上のoursのやつのコピペしてtheirsに変更

    play_situation_msg_.inplay_situation = tmp_msg;
  }
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crane::PlaySwitcher)
