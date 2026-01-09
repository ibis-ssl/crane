// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__ATTACKER_SKILL_TACTIC_HPP_
#define CRANE_TACTICS__ATTACKER_SKILL_TACTIC_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/attacker.hpp>
#include <crane_tactics/tactic_base.hpp>
#include <functional>
#include <memory>
#include <range/v3/algorithm/contains.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class AttackerSkillTactic : public TacticBase
{
public:
  std::shared_ptr<skills::Attacker> skill = nullptr;

  COMPOSITION_PUBLIC explicit AttackerSkillTactic(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  : TacticBase("AttackerSkill", world_model)
  {
  }

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
    if (robots.empty()) {
      return {TacticBase::Status::RUNNING, {}};
    }
    if (not skill) {
      skill = std::make_shared<skills::Attacker>("attacker", robots.front().id, world_model);
    }

    std::string state_name(magic_enum::enum_name(skill->getCurrentState()));
    {
      visualizer->circle()
        .center(skill->commander()->getRobot()->pose.pos)
        .radius(0.3)
        .stroke("red")
        .strokeWidth(20)
        .build();
    }
    if (world_model->ball().isMoving()) {
      {
        auto polyline_builder = visualizer->polyline();
        for (auto [point, distance] : world_model->getBallSequence(2.0, 0.1)) {
          polyline_builder.addPoint(point);
        }
        polyline_builder.stroke("orange", 0.3).strokeWidth(100).build();
      }
    }
    auto status = skill->run();
    if (skill->getID() != robots.front().id) {
      std::stringstream ss;
      ss << "スキルのIDは" << static_cast<int>(skill->getID()) << "ですが、選択されたロボットのIDは"
         << static_cast<int>(robots.front().id) << "です。";
      ss << "スキルのStateは" << magic_enum::enum_name(skill->getCurrentState()) << "です。";
      RCLCPP_WARN(rclcpp::get_logger("AttackerSkillTactic"), "%s", ss.str().c_str());
    }
    return {static_cast<TacticBase::Status>(status), {skill->getRobotCommand()}};
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    // ボールに近いロボットを優先（距離が小さいほど適している）
    auto wm = world_model;  // shared_ptrをコピー
    return
      [wm](const std::shared_ptr<RobotInfo> & robot) { return robot->getDistance(wm->ball().pos); };
  }
};

}  // namespace crane
#endif  // CRANE_TACTICS__ATTACKER_SKILL_TACTIC_HPP_
