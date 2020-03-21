// Copyright (c) 2019 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "crane_play_switcher/play_switcher.hpp"

#include "rclcpp/rclcpp.hpp"

namespace crane
{
PlaySwitcher::PlaySwitcher(const rclcpp::NodeOptions & options)
: Node("crane_play_switcher", options)
{
  RCLCPP_INFO(this->get_logger(), "PlaySwitcher is constructed.");
  auto referee_callback =
    [this](const consai2r2_msgs::msg::DecodedReferee::SharedPtr msg) -> void
    {
      this->referee_callback(msg);
    };
  auto world_model_callback =
    [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void
    {
      this->world_model_callback(msg);
    };
  // TODO トピック名を合わせている
  pub_play_situation_ = this->create_publisher<crane_msgs::msg::PlaySituation>("~/play_situation",
      10);
  sub_decoded_referee_ = this->create_subscription<consai2r2_msgs::msg::DecodedReferee>(
    "~/decoded_referee", 10,
    referee_callback);
  sub_world_model_ = this->create_subscription<crane_msgs::msg::WorldModel>("~/world_model", 10,
      world_model_callback);
}

void PlaySwitcher::referee_callback(const consai2r2_msgs::msg::DecodedReferee::SharedPtr msg)
{
  play_situation_msg_.referee_id = msg->referee_id;
  play_situation_msg_.referee_text = msg->referee_text;
  play_situation_msg_.is_inplay = msg->is_inplay;
  play_situation_msg_.placement_position = msg->placement_position;
}

void PlaySwitcher::world_model_callback(
  const crane_msgs::msg::WorldModel::SharedPtr msg)
{
  play_situation_msg_.world_model = *msg;
  if (play_situation_msg_.is_inplay) {
    crane_msgs::msg::InPlaySituation tmp_msg;
    geometry_msgs::msg::Pose2D ball_pose = play_situation_msg_.ball_info.pose;
    // geometry_msgs::msg::Pose2D ball_velocity = play_situation_msg_.ball_info.velocity;

    // FIXME velocityとaccを利用して，ボールにたどり着くまでの時間でソート
    // ボールとの距離が最小なロボットid
    // FIXME エレガントな書き方
    tmp_msg.nearest_to_ball_robot_id_ours =
      (std::min_element(play_situation_msg_.robot_info_ours.begin(),
      play_situation_msg_.robot_info_ours.end(),
      [](consai2r2_msgs::msg::RobotInfo a, consai2r2_msgs::msg::RobotInfo b) {
        return std::hypot(a.pose.x - ball_pose.x) < std::hypot(b.pose.x - ball_pose.x);
      }))->robot_id;
    tmp_msg.nearest_to_ball_robot_id_theirs =
      (std::min_element(play_situation_msg_.robot_info_theirs.begin(),
      play_situation_msg_.robot_info_theirs.end(),
      [](consai2r2_msgs::msg::RobotInfo a, consai2r2_msgs::msg::RobotInfo b) {
        return std::hypot(a.pose.x - ball_pose.x) < std::hypot(b.pose.x - ball_pose.x);
      }))->robot_id;
    // FIXME 正しく設定
    tmp_msg.ball_possession_ours = true;
    tmp_msg.ball_possession_theirs = true;

    play_situation_msg_.inplay_situation = tmp_msg;
  }
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crane::PlaySwitcher)
