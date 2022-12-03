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
    msg_.role_id = 0;
    msg_.param_id.clear();
    msg_.param_size.clear();
    msg_.unit.clear();
    msg_.offset.clear();
    msg_.param_num = 0;

    msg_.score.clear();
  }
  void addParam(int id, int size, float unit, float offset)
  {
    msg_.param_id.push_back(id);
    msg_.param_size.push_back(size);
    msg_.unit.push_back(unit);
    msg_.offset.push_back(offset);

    msg_.param_num++;
  }
  void setRoleID(uint8_t id) { msg_.role_id = id; }

  void reserveScore()
  {
    int array_size = 1;
    for (auto size : msg_.param_size) {
      array_size *= size;
    }
    msg_.score = std::vector<float>(0.0f, array_size);
  }

  float & getScore(std::vector<int> indices, float score)
  {
    int size = msg_.score.size();
    int index = 0;
    for (int i = 0; i < msg_.param_id.size(); i++) {
      size /= msg_.param_size.at(i);
      index += indices.at(i) * size;
    }
    return msg_.score.at(index);
  }

  crane_msgs::msg::RoleScore getMsg() { return msg_; }

private:
  crane_msgs::msg::RoleScore msg_;
};
#endif  // CRANE_MSG_WRAPPERS__ROLE_SCORE_WRAPPER_HPP_
