// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_
#define CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "crane_msg_wrappers/play_situation_wrapper.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/play_situation.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_play_switcher/visibility_control.h"
#include "robocup_ssl_msgs/msg/referee.hpp"

namespace crane
{

class BallAnalyzer
{
public:
  BallAnalyzer() {}

  void update(const WorldModelWrapper::SharedPtr & world_model)
  {
    bool pre_is_our_ball = is_our_ball;
    auto ball = world_model->ball.pos;

    if (is_our_ball) {
      // 敵がボールに触れたかどうか判定
      auto [nearest_robot, ball_dist] =
        world_model->getNearestRobotsWithDistanceFromPoint(ball, world_model->theirs.robots);
      if (ball_dist < 0.1) {
        is_our_ball = false;
      }
    } else {
      // 味方がボールに触れたかどうか判定
      auto [nearest_robot, ball_dist] =
        world_model->getNearestRobotsWithDistanceFromPoint(ball, world_model->ours.robots);
      if (ball_dist < 0.1) {
        is_our_ball = true;
      }
    }

    if (pre_is_our_ball != is_our_ball) {
      // TODO: ボール所有権が移動したときの処理
      //      last_changed_state.stamp = world_model->stamp;
      //      last_changed_state.ball_position = ball;
      if (is_our_ball) {
        RCLCPP_INFO(rclcpp::get_logger("crane_play_switcher"), "We got the ball!");
      } else {
        RCLCPP_INFO(rclcpp::get_logger("crane_play_switcher"), "They got the ball!");
      }
    }
  }

  void eventCallback(crane_msgs::msg::PlaySituation & play_situation)
  {
    // TODO: DIRECTなど，ボール所有権が移動するイベントの処理
  }

  bool isOurBall() { return is_our_ball; }

private:
  bool is_our_ball = false;
  bool is_passing = false;
};

class PlaySwitcher : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PlaySwitcher(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_pub;

  rclcpp::Subscription<robocup_ssl_msgs::msg::Referee>::SharedPtr decoded_referee_sub;

  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub;

  void referee_callback(const robocup_ssl_msgs::msg::Referee & msg);

  WorldModelWrapper::SharedPtr world_model;

  crane_msgs::msg::PlaySituation play_situation_msg;

  BallAnalyzer ball_analyzer;

  struct LastCommandChangedState
  {
    rclcpp::Time stamp;

    Point ball_position;

  } last_command_changed_state;
};
}  // namespace crane

#endif  // CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_
