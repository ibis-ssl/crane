// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/geometry_wrapper.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <string>
#include <vector>

namespace crane
{
struct PlaySituationWrapper
{
  struct IDWithText
  {
    uint32_t id;

    std::string text;
  };

  auto isInplay() -> bool const
  {
    return situation_command.id == crane_msgs::msg::PlaySituation::INPLAY;
  }

  Point placement_position;

  auto update(const crane_msgs::msg::PlaySituation & msg) -> void;

  auto getRefereeCommandID() -> uint32_t const
  {
    return referee_command_raw.id;
  }

  auto getRefereeCommandText() -> std::string const
  {
    return referee_command_raw.text;
  }

  auto getSituationCommandID() -> uint32_t const
  {
    return situation_command.id;
  }

  auto getSituationCommandText() -> std::string const
  {
    return situation_command.text;
  }

  static auto getRefereeCommandText(uint32_t id) -> std::string;

  static auto getRefereeCommandTextList() -> std::vector<std::string>;

  static auto getSituationCommandText(uint32_t id) -> std::string;

  static auto getSituationCommandTextList() -> std::vector<std::string>;

private:
  IDWithText referee_command_raw;

  IDWithText situation_command;
};
}  // namespace crane
#endif  // CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_
