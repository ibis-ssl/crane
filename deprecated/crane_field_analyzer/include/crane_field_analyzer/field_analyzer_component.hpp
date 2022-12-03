// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
