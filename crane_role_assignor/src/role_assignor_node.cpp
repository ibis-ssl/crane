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

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "crane_msgs/msg/role_scores.hpp"

class RoleAssignor : public rclcpp::Node
{
public:
  RoleAssignor()
  : Node("crane_rore_assignor")
  {
    subscription = this->create_subscription<crane_msgs::msg::RoleScores>(
      "topic_test",
      std::bind(&RoleAssignor::topicCallback, this, std::placeholders::_1));
    // publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
  }

private:
  void topicCallback(const crane_msgs::msg::RoleScores::SharedPtr msg)
  {
    if (msg->play_situation.is_inplay) {
      assignRoleAtInplay(msg);
    } else {
      switch (msg->play_situation.referee_id) {
        case 0:
          // HALT
          assignRoleHalt();
          break;
        case 1:
          // STOP
          assignRoleStop();
          break;
        case 3:
          // FORCE_START
          assignRoleAtForceStart(msg);
          break;
        case 11:
          // OUR_KICKOFF_PREPARATION
          assignRoleAtOurKickOffPreparation();
          break;
        case 12:
          // OUR_KICKOFF_START
          assignRoleAtOurKickOffStart();
          break;
        case 13:
          // OUR_PENALTY_PREPARATION
          assignRoleAtOurPenaltyKickPreparation();
          break;
        case 14:
          // OUR_PENALTY_START
          assignRoleAtOurPenaltyKickStart();
          break;
        case 15:
          // OUR_DIRECT_FREE
          assignRoleOurDirectFree();
          break;
        case 16:
          // OUR_INDIRECT_FREE
          assignRoleOurIndirectFree();
          break;
        case 17:
          // OUR_TIMEOUT
          assignRoleHalt();
          break;
        case 18:
          // OUR_GOAL
          assignRoleStop();
          break;
        case 19:
          // OUR_BALL_PLACEMENT
          assignRoleAtOurBallPlacement();
          break;
        case 21:
          // THEIR_KICKOFF_PREPARATION
          assignRoleStop();
          break;
        case 22:
          // THEIR_KICKOFF_START
          assignRoleStop();
          break;
        case 23:
          // THEIR_PENALTY_PREPARATION
          assignRoleAtTheirPenalty();
          break;
        case 24:
          // THEIR_PENALTY_START
          assignRoleAtTheirPenalty();
          break;
        case 25:
          // THEIR_DIRECT_FREE
          assignRoleAtTheirSetPlay();
          break;
        case 26:
          // THEIR_INDIRECT_FREE
          assignRoleAtTheirSetPlay();
          break;
        case 27:
          // THEIR_TIMEOUT
          assignRoleStop();
          break;
        case 28:
          // THEIR_GOAL
          assignRoleStop();
          break;
        case 29:
          // THEIR_BALL_PLACEMENT
          assignRoleAtTheirBallPlacement();
          break;

        default:
          assignRoleHalt();
          break;
      }
    }
  }

  rclcpp::Subscription<crane_msgs::msg::RoleScores>::SharedPtr subscription;

  void assignRoleAtInplay(const crane_msgs::msg::RoleScores::SharedPtr msg)
  {
    // inplay時のRole割当て
    auto situation = msg->play_situation.inplay_situation;
    auto our_robots = msg->world_model.robot_info_ours;
    if (situation.ball_possession_ours == true && situation.ball_possession_theirs == true) {
      // 攻撃
      assignAttackPriorityRole();
    }else if (situation.ball_possession_ours == true && situation.ball_possession_theirs == false){
      // 攻撃
      assignAttackPriorityRole();
    }else if (situation.ball_possession_ours == false && situation.ball_possession_theirs == true){
      // 防御
      assignDefensePriorityRole();
    }else if (situation.ball_possession_ours == false && situation.ball_possession_theirs == false){
      // 攻撃
      assignAttackPriorityRole();
    }else {
      // 攻撃
      assignAttackPriorityRole();
    }
  }

  void assignAttackPriorityRole()
  {
    // 攻撃優先のRole割当て
  }

  void assignDefensePriorityRole()
  {
    // 防御優先のRole割当て
  }

  void assignRoleHalt()
  {
    // すべてのロボットのRoleをHaltにする
    // 停止
  }

  void assignRoleStop()
  {
    // すべてのロボットのRoleをStopにする
    // 指定の位置に移動
  }

  void assignRoleAtForceStart(const crane_msgs::msg::RoleScores::SharedPtr msg)
  {
    // force startのときのRole割当て。インプレイのときと同じ動きをする。
    assignRoleAtInplay(msg);
  }

  void assignRoleAtOurKickOffPreparation()
  {
    // 自チームのキックオフの準備
  }

  void assignRoleAtOurKickOffStart()
  {
    // 自チームのキックオフの開始
  }

  void assignRoleAtOurPenaltyKickPreparation()
  {
    // 自チームのPKの準備
  }

  void assignRoleAtOurPenaltyKickStart()
  {
    // 自チームのPKスタート
  }

  void assignRoleOurDirectFree()
  {
    // 自チームの直接フリーキック
  }

  void assignRoleOurIndirectFree()
  {
    // 自チームの間接フリーキック
  }

  void assignRoleAtOurBallPlacement()
  {
    // 自チームのボールプレイスメント
  }

  void assignRoleAtTheirPenalty()
  {
    // 敵のペナルティーキック
    // ゴーリー以外は一列に並ぶ
  }

  void assignRoleAtTheirSetPlay()
  {
    // 敵のセットプレイのときの待機位置
    // atackerは指定の指定の位置に移動
  }

  void assignRoleAtTheirBallPlacement()
  {
    // 敵のボールプレイスメント。邪魔をしない位置に移動
  }
};
int main()
{
  std::cout << "hello, this is role_assignor_node" << std::endl;
  return 0;
}
