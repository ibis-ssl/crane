// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__ROLE_SCORE_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__ROLE_SCORE_WRAPPER_HPP_

#include <string>
#include <vector>

#include "crane_msgs/msg/role_score.hpp"

struct RoleScoreWrapper
{
  RoleScoreWrapper() { init(); }
  void init()
  {
    msg.role_id = 0;
    msg.param_id.clear();
    msg.param_size.clear();
    msg.unit.clear();
    msg.offset.clear();
    msg.param_num = 0;

    msg.score.clear();
  }
  void addParam(int id, int size, float unit, float offset)
  {
    msg.param_id.push_back(id);
    msg.param_size.push_back(size);
    msg.unit.push_back(unit);
    msg.offset.push_back(offset);

    msg.param_num++;
  }
  void setRoleID(uint8_t id) { msg.role_id = id; }

  void reserveScore()
  {
    int array_size = 1;
    for (auto size : msg.param_size) {
      array_size *= size;
    }
    msg.score = std::vector<float>(0.0f, array_size);
  }

  float & getScore(std::vector<int> indices, float score)
  {
    int size = msg.score.size();
    int index = 0;
    for (int i = 0; i < msg.param_id.size(); i++) {
      size /= msg.param_size.at(i);
      index += indices.at(i) * size;
    }
    return msg.score.at(index);
  }

  crane_msgs::msg::RoleScore getMsg() { return msg; }

private:
  crane_msgs::msg::RoleScore msg;
};
#endif  // CRANE_MSG_WRAPPERS__ROLE_SCORE_WRAPPER_HPP_
