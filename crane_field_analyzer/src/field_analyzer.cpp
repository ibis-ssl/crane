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

#include "crane_field_analyzer/field_analyzer.hpp"

#include "rclcpp/rclcpp.hpp"


namespace crane
{
FieldAnalyzer::FieldAnalyzer(const rclcpp::NodeOptions & options)
: Node("crane_field_analyzer", options)
{
  RCLCPP_INFO(this->get_logger(), "FieldAnalyzer is constructed.");
  auto play_situation_callback =
    [this](const crane_msgs::msg::PlaySituation::SharedPtr msg) -> void
    {
      this->play_situation_callback(msg);
    };
  // FIXME トピック名を合わせる
  pub_role_scores_ = this->create_publisher<crane_msgs::msg::RoleScores>("~/role_scores",
      10);
  sub_play_situation_ = this->create_subscription<crane_msgs::msg::PlaySituation>(
    "crane_play_switcher/play_situation", 10,
    play_situation_callback);
}

void FieldAnalyzer::play_situation_callback(const crane_msgs::msg::PlaySituation::SharedPtr msg)
{
  crane_msgs::msg::RoleScores role_scores_msg{};
  if (msg.is_inplay) {
    crane_msgs::msg::RoleScore score;
    score.role_id = crane_msgs::msg::RoleScore::DEFENDER;
    score.param_num = 4;
    // FIXME 多分crane_msgsにRoleごとにパラメータ名enumを定義するらしい
    score.param_id.emplace_back(0);   // 後でcrane_msgs::msg::Defender::IDとなるべきもの
    score.param_size.emplace_back(msg.world_model.robot_info_ours.size());   // ロボットの数
    score.unit.emplace_back(1);
  }
  // FIXME 自チームボールと敵チームボールのフラグは両方trueということもありうるが，その場合分けをどうするか
  if (msg.is_inplay && msg.in_play_situation.ball_possession_ours) {
    role_candidate.emplace_back(crane_msgs::msg::RoleScore::PASSER);
  }
  if (msg.is_inplay && msg.in_play_situation.ball_possession_theirs) {
    role_candidate.emplace_back(crane_msgs::msg::RoleScore::PASSCUTTER);
    role_candidate.emplace_back(crane_msgs::msg::RoleScore::BALLSTEALER);
  }
  if (msg.referee_id == crane_msgs::msg::PlaySituation::OUR_BALL_PLACEMENT) {
    role_candidate.emplace_back(crane_msgs::msg::RoleScore::BALLPLACER);
    role_candidate.emplace_back(crane_msgs::msg::RoleScore::NOTBALLPLACER);
  }
  if (msg.referee_id == crane_msgs::msg::PlaySituation::HALT ||
    msg.referee_id == crane_msgs::msg::PlaySituation::STOP)
  {
    role_candidate.emplace_back(crane_msgs::msg::RoleScore::IDLER);
  }

  pub_role_scores.publish(role_scores_msg);
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crane::FieldAnalyzer)