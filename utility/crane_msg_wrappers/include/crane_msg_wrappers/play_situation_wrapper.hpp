// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_

#include <string>

#include "crane_msg_wrappers/geometry_wrapper.hpp"
#include "crane_msg_wrappers/in_play_situation_wrapper.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "crane_msgs/msg/play_situation.hpp"

struct PlaySituationWrapper
{
  struct RefereeInfo
  {
    uint8_t id;
    std::string text;
  } referee;

  bool is_inplay;
  InPlaySituationWrapper inplay_situation;
  Eigen::Vector2f placement_position;
  WorldModelWrapper world_model;
  void update(const crane_msgs::msg::PlaySituation & msg)
  {
    referee.id = msg.referee_id;
    referee.text = msg.referee_text;

    is_inplay = msg.is_inplay;
    inplay_situation.update(msg.inplay_situation);
    placement_position << msg.placement_position.x, msg.placement_position.y;
    //    auto b = msg.world_model;
    //    auto a = std::make_shared<crane_msgs::msg::WorldModel>();
    //    *a = msg.world_model;
    world_model.update(msg.world_model);
  }
};
#endif  // CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_
