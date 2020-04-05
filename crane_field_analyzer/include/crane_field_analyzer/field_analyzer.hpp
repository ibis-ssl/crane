// Copyright (c) 2019 ibis-ssl
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

#ifndef CRANE_FIELD_ANALYZER__FIELD_ANALYZER_HPP_
#define CRANE_FIELD_ANALYZER__FIELD_ANALYZER_HPP_

#include "crane_field_analyzer/visibility_control.h"
#include "crane_msgs/msg/play_situation.hpp"
#include "crane_msgs/msg/role_scores.hpp"
#include "rclcpp/rclcpp.hpp"

namespace crane
{
class FieldAnalyzer : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit FieldAnalyzer(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<crane_msgs::msg::RoleScores>::SharedPtr pub_role_scores_;
  rclcpp::Subscription < crane_msgs::msg::PlaySituation::SharedPtr sub_play_situation_;

  void play_situation_callback(const crane_msgs::msg::PlaySituation::SharedPtr msg);
};
}  // namespace crane


#endif  // CRANE_FIELD_ANALYZER__FIELD_ANALYZER_HPP_
