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

#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "crane_game_analyzer/game_analyzer.hpp"

namespace crane
{
GameAnalyzer::GameAnalyzer(const rclcpp::NodeOptions & options)
: Node("crane_game_analyzer", options)
{
  RCLCPP_INFO(this->get_logger(), "GameAnalyzer is constructed.");

  auto world_model_callback =
    [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void
    {
      this->world_model_callback(msg);
    };
  // FIXME トピック名を合わせる
  sub_world_model_ = this->create_subscription<crane_msgs::msg::WorldModel>(
    "crane_world_observer/world_model", 10,
    world_model_callback);
}

double calcDistanceFromBall(
  const consai2r2_msgs::msg::RobotInfo & robot_info,
  const geometry_msgs::msg::Pose2D & ball_pose)
{
  return std::hypot(robot_info.pose.x - ball_pose.x, robot_info.pose.y - ball_pose.y);
}

void GameAnalyzer::world_model_callback(
  const crane_msgs::msg::WorldModel::SharedPtr msg)
{
  play_situation_msg_.world_model = *msg;
  if (play_situation_msg_.is_inplay) {
    crane_msgs::msg::InPlaySituation tmp_msg;
    geometry_msgs::msg::Pose2D ball_pose = play_situation_msg_.world_model.ball_info.pose;
    // geometry_msgs::msg::Pose2D ball_velocity
    //   = play_situation_msg_.world_model.ball_info.velocity;

    // ボールとの距離が最小なロボットid
    // FIXME velocityとaccを利用して，ボールにたどり着くまでの時間でソート
    const std::vector<consai2r2_msgs::msg::RobotInfo> & ours =
      play_situation_msg_.world_model.robot_info_ours;
    auto nearest_ours_itr = std::min_element(ours.begin(), ours.end(),
        [ball_pose](consai2r2_msgs::msg::RobotInfo a, consai2r2_msgs::msg::RobotInfo b) {
          return calcDistanceFromBall(a, ball_pose) < calcDistanceFromBall(b, ball_pose);
        });
    tmp_msg.nearest_to_ball_robot_id_ours = nearest_ours_itr->robot_id;
    double shortest_distance_ours = calcDistanceFromBall(*nearest_ours_itr, ball_pose);

    // FIXME 所持判定距離閾値を可変にする
    tmp_msg.ball_possession_ours = shortest_distance_ours < 1.0e-3;

    // ここから上のoursのやつのコピペしてtheirsに変更
    const std::vector<consai2r2_msgs::msg::RobotInfo> & theirs =
      play_situation_msg_.world_model.robot_info_theirs;
    auto nearest_theirs_itr = std::min_element(theirs.begin(), theirs.end(),
        [ball_pose](consai2r2_msgs::msg::RobotInfo a, consai2r2_msgs::msg::RobotInfo b) {
          return calcDistanceFromBall(a, ball_pose) < calcDistanceFromBall(b, ball_pose);
        });
    tmp_msg.nearest_to_ball_robot_id_theirs = nearest_theirs_itr->robot_id;
    double shortest_distance_theirs = calcDistanceFromBall(*nearest_theirs_itr, ball_pose);

    // FIXME 所持判定距離閾値を可変にする
    tmp_msg.ball_possession_theirs = shortest_distance_theirs < 1.0e-3;
    // ここまで上のoursのやつのコピペしてtheirsに変更

    play_situation_msg_.inplay_situation = tmp_msg;
  }
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crane::GameAnalyzer)
