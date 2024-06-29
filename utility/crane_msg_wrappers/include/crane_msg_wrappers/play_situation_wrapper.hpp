// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_

#include <crane_basics/boost_geometry.hpp>
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

  auto isInplay() const -> bool
  {
    return situation_command.id >= crane_msgs::msg::PlaySituation::OUR_INPLAY;
  }

  Point placement_position;

  auto update(const crane_msgs::msg::PlaySituation & msg) -> void;

  auto getRefereeCommandID() const -> uint32_t { return referee_command_raw.id; }

  auto getRefereeCommandText() const -> std::string { return referee_command_raw.text; }

  auto getSituationCommandID() const -> uint32_t { return situation_command.id; }

  auto getSituationCommandText() const -> std::string { return situation_command.text; }

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
