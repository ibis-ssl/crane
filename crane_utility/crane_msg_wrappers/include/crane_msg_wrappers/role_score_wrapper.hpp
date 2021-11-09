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

#ifndef CRANE_MSG_WRAPPERS__ROLE_SCORE_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__ROLE_SCORE_WRAPPER_HPP_

#include <string>
#include <vector>
#include "crane_msgs/msg/role_score.hpp"

struct RoleScoreWrapper
{
  RoleScoreWrapper()
  {
    init();
  }
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
  void setRoleID(uint8_t id)
  {
    msg_.role_id = id;
  }

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

  crane_msgs::msg::RoleScore getMsg()
  {
    return msg_;
  }

private:
  crane_msgs::msg::RoleScore msg_;
};
#endif  // CRANE_MSG_WRAPPERS__ROLE_SCORE_WRAPPER_HPP_
