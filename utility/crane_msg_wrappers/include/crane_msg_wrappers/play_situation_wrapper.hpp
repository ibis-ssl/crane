// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msgs/msg/named_int.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <string>
#include <vector>

namespace crane
{
auto getStageText(uint32_t id) -> std::string;

auto getStageNamedInt(uint32_t id) -> crane_msgs::msg::NamedInt;

auto getStageTextList() -> std::vector<std::string>;

auto getRefereeCommandText(uint32_t id) -> std::string;

auto getRefereeCommandNamedInt(uint32_t id) -> crane_msgs::msg::NamedInt;

auto getRefereeCommandTextList() -> std::vector<std::string>;

auto getSituationCommandText(uint32_t id) -> std::string;

auto getSituationCommandNamedInt(uint32_t id) -> crane_msgs::msg::NamedInt;

auto getSituationCommandTextList() -> std::vector<std::string>;

// STOP/セットプレイ中（INPLAY/HALT/HALF_TIME/POST_GAME以外）に相手PAへの拡大マージンが必要か
// SSL Rule 5.2.4: STOP・フリーキック中は攻撃側ロボットが相手PAから0.2m以上離れていなければならない
inline auto needsExpandedPenaltyAreaOffset(uint8_t cmd) -> bool
{
  using PS = crane_msgs::msg::PlaySituation;
  switch (cmd) {
    case PS::INPLAY:
    case PS::HALT:
    case PS::HALF_TIME:
    case PS::POST_GAME:
      return false;
    default:
      return true;
  }
}
}  // namespace crane
#endif  // CRANE_MSG_WRAPPERS__PLAY_SITUATION_WRAPPER_HPP_
