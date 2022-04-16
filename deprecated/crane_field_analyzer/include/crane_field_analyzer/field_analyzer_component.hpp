// Copyright (c) 2022 ibis-ssl
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

#ifndef CRANE_FIELD_ANALYZER__FIELD_ANALYZER_COMPONENT_HPP_
#define CRANE_FIELD_ANALYZER__FIELD_ANALYZER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"

#include "crane_msgs/msg/play_situation.hpp"
#include "crane_msg_wrappers/play_situation_wrapper.hpp"
#include "crane_msgs/msg/role_scores.hpp"
#include "crane_msg_wrappers/role_score_wrapper.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_msg_wrappers/world_model_wrapper.hpp"

#include "crane_field_analyzer/visibility_control.h"

namespace crane
{
class FieldAnalyzerComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit FieldAnalyzerComponent(const rclcpp::NodeOptions& options);

private:
  crane_msgs::msg::RoleScores role_scores_{};
  PlaySituationWrapper play_situation_;
  //  RoleScoreWrapper role_scores_;
  WorldModelWrapper world_model_;
  rclcpp::Publisher<crane_msgs::msg::RoleScores>::SharedPtr pub_role_scores_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;
  rclcpp::Subscription<crane_msgs::msg::PlaySituation>::SharedPtr sub_play_situation_;

  void world_model_callback(const crane_msgs::msg::WorldModel& msg);
  void play_situation_callback(const crane_msgs::msg::PlaySituation::SharedPtr msg);
};
}  // namespace crane

#endif  // CRANE_FIELD_ANALYZER__FIELD_ANALYZER_COMPONENT_HPP_
