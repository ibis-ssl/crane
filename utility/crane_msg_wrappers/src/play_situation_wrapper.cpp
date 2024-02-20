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
#define CMD_STRING_MAPPING(TYPE, CMD) \
  {                                   \
    TYPE::CMD, #CMD                   \
  }

static std::map<int, std::string> referee_command_map = {
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_HALT),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_STOP),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_NORMAL_START),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_FORCE_START),
  CMD_STRING_MAPPING(
    robocup_ssl_msgs::msg::Referee, COMMAND_PREPARE_KICKOFF_YELLOW),
  CMD_STRING_MAPPING(
    robocup_ssl_msgs::msg::Referee, COMMAND_PREPARE_KICKOFF_BLUE),
  CMD_STRING_MAPPING(
    robocup_ssl_msgs::msg::Referee, COMMAND_PREPARE_PENALTY_YELLOW),
  CMD_STRING_MAPPING(
    robocup_ssl_msgs::msg::Referee, COMMAND_PREPARE_PENALTY_BLUE),
  CMD_STRING_MAPPING(
    robocup_ssl_msgs::msg::Referee, COMMAND_DIRECT_FREE_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_DIRECT_FREE_BLUE),
  CMD_STRING_MAPPING(
    robocup_ssl_msgs::msg::Referee, COMMAND_INDIRECT_FREE_YELLOW),
  CMD_STRING_MAPPING(
    robocup_ssl_msgs::msg::Referee, COMMAND_INDIRECT_FREE_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_TIMEOUT_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_TIMEOUT_BLUE),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_GOAL_YELLOW),
  CMD_STRING_MAPPING(robocup_ssl_msgs::msg::Referee, COMMAND_GOAL_BLUE),
  CMD_STRING_MAPPING(
    robocup_ssl_msgs::msg::Referee, COMMAND_BALL_PLACEMENT_YELLOW),
  CMD_STRING_MAPPING(
    robocup_ssl_msgs::msg::Referee, COMMAND_BALL_PLACEMENT_BLUE)};

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
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_INDIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_INDIRECT_FREE),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, OUR_BALL_PLACEMENT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, THEIR_BALL_PLACEMENT),
  CMD_STRING_MAPPING(crane_msgs::msg::PlaySituation, INPLAY)};

auto PlaySituationWrapper::update(const crane_msgs::msg::PlaySituation & msg)
  -> void
{
  referee_command_raw.id = msg.command_raw;
  referee_command_raw.text = referee_command_map[msg.command_raw];

  situation_command.id = msg.command;
  situation_command.text = situation_command_map[msg.command];

  placement_position << msg.placement_position.x, msg.placement_position.y;
}

auto PlaySituationWrapper::getRefereeCommandText(uint32_t id) -> std::string
{
  return referee_command_map[id];
}

auto PlaySituationWrapper::getRefereeCommandTextList()
  -> std::vector<std::string>
{
  std::vector<std::string> ret;
  for (auto & [id, text] : referee_command_map) {
    ret.push_back(text);
  }
  return ret;
}

auto PlaySituationWrapper::getSituationCommandText(uint32_t id) -> std::string
{
  return situation_command_map[id];
}

auto PlaySituationWrapper::getSituationCommandTextList()
  -> std::vector<std::string>
{
  std::vector<std::string> ret;
  for (auto & [id, text] : situation_command_map) {
    ret.push_back(text);
  }
  return ret;
}

}  // namespace crane
