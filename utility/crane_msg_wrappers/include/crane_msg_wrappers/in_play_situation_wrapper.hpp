// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__IN_PLAY_SITUATION_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__IN_PLAY_SITUATION_WRAPPER_HPP_

#include "crane_msgs/msg/in_play_situation.hpp"

struct InPlaySituationWrapper
{
  struct NearestToBallRobotID
  {
    uint8_t id_ours;
    uint8_t id_theirs;
  } nearest_to_ball;

  struct BallPossession
  {
    bool ours;
    bool theirs;
  } ball_possession;

  void update(const crane_msgs::msg::InPlaySituation & msg)
  {
    nearest_to_ball.id_ours = msg.nearest_to_ball_robot_id_ours;
    nearest_to_ball.id_theirs = msg.nearest_to_ball_robot_id_theirs;

    ball_possession.ours = msg.ball_possession_ours;
    ball_possession.theirs = msg.ball_possession_theirs;
  }
};
#endif  // CRANE_MSG_WRAPPERS__IN_PLAY_SITUATION_WRAPPER_HPP_
