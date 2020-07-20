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

#ifndef CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_

#include "crane_msgs/msg/play_situation.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msg_wrappers/geometry_wrapper.hpp"
#include "crane_msg_wrappers/in_play_situation_wrapper.hpp"

struct PlaySituation
{
  struct RefereeInfo {
    uint8_t id;
    std::string text;
  } referee;

  bool is_inplay;
  InPlaySituation inplay_situation;
  Eigen::Vector2f placement_position;
  WorldModel world_model;
  void update(const crane_msgs::msg::PlaySituation &msg)
  {
    referee.id = msg.referee_id;
    referee.text = msg.referee_text;

    is_inplay = msg.is_inplay;
    inplay_situation.update(msg.inplay_situation);
    placement_position << msg.placement_position.x, msg.placement_position.y;
    world_model.update(msg.world_model);
  }
};
#endif  // CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_
