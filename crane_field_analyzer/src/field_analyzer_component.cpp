// Copyright (c) 2020 ibis-ssl
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

#include "rclcpp/rclcpp.hpp"
#include "crane_msgs/msg/sub_role.hpp"
#include "crane_field_analyzer/field_analyzer_component.hpp"

namespace crane
{
FieldAnalyzerComponent::FieldAnalyzerComponent(const rclcpp::NodeOptions & options)
: Node("crane_field_analyzer", options)
{
  RCLCPP_INFO(this->get_logger(), "FieldAnalyzer is constructed.");
  auto world_model_callback =
    [this](const crane_msgs::msg::WorldModel::SharedPtr msg) -> void
    {
      this->world_model_callback(msg);
    };
  auto play_situation_callback =
    [this](const crane_msgs::msg::PlaySituation::SharedPtr msg) -> void
    {
      this->play_situation_callback(msg);
    };
  // FIXME トピック名を合わせる
  pub_role_scores_ = this->create_publisher<crane_msgs::msg::RoleScores>("~/role_scores",
      10);
  sub_world_model_ = this->create_subscription<crane_msgs::msg::WorldModel>(
    "crane_world_observer/world_model ", 10,
    world_model_callback);
  sub_play_situation_ = this->create_subscription<crane_msgs::msg::PlaySituation>(
    "crane_play_switcher/play_situation", 10,
    play_situation_callback);
}

void FieldAnalyzerComponent::world_model_callback(const crane_msgs::msg::WorldModel::SharedPtr msg)
{
  role_scores_.world_model = *msg;
  world_model_.update(msg);
}

void FieldAnalyzerComponent::play_situation_callback(
  const crane_msgs::msg::PlaySituation::SharedPtr msg)
{
  using RoleScore = crane_msgs::msg::RoleScore;
  using PlaySituation = crane_msgs::msg::PlaySituation;
  using SubRole = crane_msgs::msg::SubRole;

  role_scores_.play_situation = *msg;

  // ---- INPLAY ---- //
  if (msg->is_inplay) {
    RoleScore score;
    score.role_id = RoleScore::DEFENDER;
    score.param_num = 4;
    // FIXME 多分crane_msgsにRoleごとにパラメータ名enumを定義するらしい
    score.param_id.emplace_back(SubRole::GOALIE);  // 後でcrane_msgs::msg::Defender::IDとなるべきもの
    score.param_size.emplace_back(msg->world_model.robot_info_ours.size());   // ロボットの数
    score.unit.emplace_back(1);
    role_scores_.role_scores.emplace_back(score);
  }
  // FIXME 自チームボールと敵チームボールのフラグは両方trueということもありうる
  // その場合分けをどうするか
  if (msg->is_inplay && msg->inplay_situation.ball_possession_ours) {
    RoleScore score;
    score.role_id = RoleScore::PASSER;
    role_scores_.role_scores.emplace_back(score);
  }
  if (msg->is_inplay && msg->inplay_situation.ball_possession_theirs) {
    RoleScore score;
    score.role_id = RoleScore::PASSCUTTER;
    role_scores_.role_scores.emplace_back(score);
    score.role_id = RoleScore::BALLSTEALER;
    role_scores_.role_scores.emplace_back(score);
  }

  if (msg->referee_id == PlaySituation::OUR_BALL_PLACEMENT) {
    RoleScore score;
    score.role_id = RoleScore::BALLPLACER;
    role_scores_.role_scores.emplace_back(score);
    score.role_id = RoleScore::NOTBALLPLACER;
    role_scores_.role_scores.emplace_back(score);
  }
  if (msg->referee_id == PlaySituation::HALT ||
    msg->referee_id == PlaySituation::STOP)
  {
    RoleScore score;
    score.role_id = RoleScore::IDLER;
    role_scores_.role_scores.emplace_back(score);
  }

  pub_role_scores_->publish(role_scores_);
}

}  // namespace crane

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(crane::FieldAnalyzerComponent)
