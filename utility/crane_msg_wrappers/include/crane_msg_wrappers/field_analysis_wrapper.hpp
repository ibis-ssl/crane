// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__FIELD_ANALYSIS_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__FIELD_ANALYSIS_WRAPPER_HPP_

#include <crane_msgs/msg/field_analysis.hpp>
#include <string>

struct FieldAnalysisWrapper
{
  std::string name;
  float unit;
  Eigen::Vector2d origin;
  int size_x;
  int size_y;

  void update(const crane_msgs::msg::FieldAnalysis & msg) {}
};
#endif  // CRANE_MSG_WRAPPERS__FIELD_ANALYSIS_WRAPPER_HPP_
