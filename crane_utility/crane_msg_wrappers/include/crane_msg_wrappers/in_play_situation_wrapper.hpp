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

#ifndef CRANE_MSG_WRAPPERS__IN_PLAY_SITUATION_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__IN_PLAY_SITUATION_WRAPPER_HPP_

#include "crane_msgs/msg/in_play_situation.hpp"

struct InPlaySituation
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
