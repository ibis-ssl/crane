// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/play_situation_wrapper.hpp"

#include <map>
#include <string>

#include "crane_msg_wrappers/world_model_wrapper.hpp"
#include "robocup_ssl_msgs/msg/referee.hpp"

namespace crane
{
#define CMD_STRING_MAPPING(TYPE, CMD) {TYPE::CMD, #CMD}

static std::map<int, std::string> stage_map = {
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, NORMAL_FIRST_HALF_PRE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, NORMAL_FIRST_HALF),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, NORMAL_HALF_TIME),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, NORMAL_SECOND_HALF_PRE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, NORMAL_SECOND_HALF),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, EXTRA_TIME_BREAK),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, EXTRA_FIRST_HALF_PRE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, EXTRA_FIRST_HALF),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, EXTRA_HALF_TIME),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, EXTRA_SECOND_HALF_PRE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, EXTRA_SECOND_HALF),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, PENALTY_SHOOTOUT_BREAK),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, PENALTY_SHOOTOUT),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeStage, POST_GAME)};

static std::map<int, std::string> referee_command_map = {
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, HALT),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, STOP),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, NORMAL_START),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, FORCE_START),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, PREPARE_KICKOFF_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, PREPARE_KICKOFF_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, PREPARE_PENALTY_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, PREPARE_PENALTY_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, DIRECT_FREE_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, DIRECT_FREE_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, INDIRECT_FREE_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, INDIRECT_FREE_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, TIMEOUT_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, TIMEOUT_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, GOAL_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, GOAL_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, BALL_PLACEMENT_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::RefereeCommand, BALL_PLACEMENT_BLUE)};

static std::map<int, std::string> situation_command_map = {
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, HALT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, STOP),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_KICKOFF_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_KICKOFF_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_KICKOFF_START),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_KICKOFF_START),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_PENALTY_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_PENALTY_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_PENALTY_START),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_PENALTY_START),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_DIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_DIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_BALL_PLACEMENT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_BALL_PLACEMENT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_TIMEOUT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_TIMEOUT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, INJECTION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, INPLAY),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_INPLAY),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_INPLAY),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, AMBIGUOUS_INPLAY),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, STOP_PRE_OUR_PENALTY_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, STOP_PRE_THEIR_PENALTY_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, STOP_PRE_OUR_KICKOFF_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, STOP_PRE_THEIR_KICKOFF_PREPARATION),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, STOP_PRE_OUR_DIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, STOP_PRE_THEIR_DIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, STOP_PRE_FORCE_START),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, HALF_TIME),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, POST_GAME)};

auto getStageText(uint32_t id) -> std::string { return stage_map[id]; }

auto getStageNamedInt(uint32_t id) -> crane_msgs::msg::NamedInt
{
  crane_msgs::msg::NamedInt named_int;
  named_int.name = getStageText(id);
  named_int.value = id;
  return named_int;
}

auto getStageTextList() -> std::vector<std::string>
{
  return stage_map | ranges::views::transform([](const auto & elem) { return elem.second; }) |
         ranges::to<std::vector>();
}

auto getRefereeCommandText(uint32_t id) -> std::string { return referee_command_map[id]; }

auto getRefereeCommandNamedInt(uint32_t id) -> crane_msgs::msg::NamedInt
{
  crane_msgs::msg::NamedInt named_int;
  named_int.name = getRefereeCommandText(id);
  named_int.value = id;
  return named_int;
}

auto getRefereeCommandTextList() -> std::vector<std::string>
{
  return referee_command_map |
         ranges::views::transform([](const auto & elem) { return elem.second; }) |
         ranges::to<std::vector>();
}

auto getSituationCommandText(uint32_t id) -> std::string { return situation_command_map[id]; }

auto getSituationCommandNamedInt(uint32_t id) -> crane_msgs::msg::NamedInt
{
  crane_msgs::msg::NamedInt named_int;
  named_int.name = getSituationCommandText(id);
  named_int.value = id;
  return named_int;
}

auto getSituationCommandTextList() -> std::vector<std::string>
{
  return situation_command_map |
         ranges::views::transform([](const auto & elem) { return elem.second; }) |
         ranges::to<std::vector>();
}

}  // namespace crane
