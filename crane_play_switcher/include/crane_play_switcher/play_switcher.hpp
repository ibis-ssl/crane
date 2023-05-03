// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_
#define CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_

#include "crane_msgs/msg/play_situation.hpp"
#include "crane_msgs/msg/world_model.hpp"
#include "crane_play_switcher/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include "robocup_ssl_msgs/msg/referee.hpp"
#<include "crane_msg_wrappers/world_model_wrapper.hpp"

namespace crane
{
class PlaySwitcher : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PlaySwitcher(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<crane_msgs::msg::PlaySituation>::SharedPtr pub_play_situation_;
  rclcpp::Subscription<robocup_ssl_msgs::msg::Referee>::SharedPtr sub_decoded_referee_;
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr sub_world_model_;

  void referee_callback(const robocup_ssl_msgs::msg::Referee::SharedPtr msg);

  void referee_diff_callback();

  void world_model_callback(const crane_msgs::msg::WorldModel & msg);

  WorldModelWrapper::UniquePtr world_model_;

  crane_msgs::msg::PlaySituation play_situation_msg_;

  struct LastCommandChangedState{
    rclcpp::Time stamp;
    Point ball_position;
  } last_command_changed_state_;
};
}  // namespace crane

#endif  // CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_
