// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__EMPLACE_ROBOT_TACTIC_HPP_
#define CRANE_TACTICS__EMPLACE_ROBOT_TACTIC_HPP_

#include <algorithm>
#include <chrono>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/emplace_robot.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <speak_ros_interfaces/action/speak.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class EmplaceRobotTactic : public TacticBase
{
public:
  using Speak = speak_ros_interfaces::action::Speak;
  using GoalHandleSpeak = rclcpp_action::ClientGoalHandle<Speak>;

  COMPOSITION_PUBLIC explicit EmplaceRobotTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node);

  ~EmplaceRobotTactic() override
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override;

  bool isHardConstraint() const override;

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override;

protected:
  void onRobotsChanged() override;

private:
  std::unordered_map<uint8_t, std::shared_ptr<skills::EmplaceRobot>> m_skill_map;

  // 音声アナウンス制御用
  rclcpp_action::Client<Speak>::SharedPtr speak_client_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface_;
  std::chrono::steady_clock::time_point last_announce_time_{};
  static constexpr std::chrono::milliseconds ANNOUNCE_INTERVAL{5000};  // 5秒間隔
  bool use_voice_announcement_{true};  // true: 音声アナウンス, false: ビープ音

  // ビープ音制御用
  std::string beep_sound_path_;

  auto sendSpeakGoal(const std::string & text) -> void;
  auto findAvailableSoundFile() -> std::string;
};

}  // namespace crane
#endif  // CRANE_TACTICS__EMPLACE_ROBOT_TACTIC_HPP_
