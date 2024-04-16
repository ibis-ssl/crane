// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_
#define CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_

#include <crane_msg_wrappers/play_situation_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/play_situation.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robocup_ssl_msgs/msg/referee.hpp>
#include <string>

#include "visibility_control.h"

namespace crane
{
class PlaySwitcher : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PlaySwitcher(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<crane_msgs::msg::PlaySituation>::SharedPtr play_situation_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr process_time_pub;

  rclcpp::Subscription<robocup_ssl_msgs::msg::Referee>::SharedPtr decoded_referee_sub;

  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr world_model_sub;

  void referee_callback(const robocup_ssl_msgs::msg::Referee & msg);

  WorldModelWrapper::SharedPtr world_model;

  crane_msgs::msg::PlaySituation play_situation_msg;

  std::string team_name = "ibis";

  struct LastCommandChangedState
  {
    rclcpp::Time stamp;

    Point ball_position;
  } last_command_changed_state;
};
}  // namespace crane

#endif  // CRANE_PLAY_SWITCHER__PLAY_SWITCHER_HPP_
