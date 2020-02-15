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

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "crane_msgs/msg/role_scores.hpp"
#include "crane_msgs/msg/world_model.hpp"

class RoleAssignor : public rclcpp::Node{
public:
  RoleAssignor() : Node("crane_rore_assignor"){
    subscription_ = this->create_subscription<crane_msgs::msg::RoleScores>(
    "topic_test",
    std::bind(&RoleAssignor::topicCallback, this, std::placeholders::_1));
  }

private:
  void topicCallback(const crane_msgs::msg::RoleScores::SharedPtr msg){
    auto situation = msg->play_situation.inplay_situation;
    if (situation.ball_possession_ours == true && situation.ball_possession_theirs == true){
        //攻撃
        assignAttackPriorityRole()
    }
    else if(situation.ball_possession_ours == true && situation.ball_possession_theirs == false){
        //攻撃
    }
    else if(situation.ball_possession_ours == false && situation.ball_possession_theirs == true){
        //防御
    }
    else if(situation.ball_possession_ours == false && situation.ball_possession_theirs == false){
        //攻撃
    }
    else{

    }
    
  }
  
  rclcpp::Subscription<crane_msgs::msg::RoleScores>::SharedPtr subscription_;

  void assignAttackPriorityRole(){

  }

  void assignDefensePriorityRole(){
    
  }
};
int main()
{
  std::cout << "hello, this is role_assignor_node" << std::endl;
  return 0;
}
