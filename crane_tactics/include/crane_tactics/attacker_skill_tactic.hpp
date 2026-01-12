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
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node & node)
  : TacticBase("attacker_skill", world_model)
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
          (void)polyline_builder.addPoint(point);
        }
        polyline_builder.stroke("orange", 0.3).strokeWidth(100).build();
      }
    }
    auto status = skill->run();
    return {static_cast<TacticBase::Status>(status), {skill->getRobotCommand()}};
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;  // shared_ptrをコピー
    auto game_analysis = getGameAnalysis();  // GameAnalysisをコピー

    // デバッグ用：推奨ロボットIDをログ出力
    static int last_logged_id = -999;
    if (game_analysis.recommended_attacker_id != last_logged_id) {
      RCLCPP_INFO(
        rclcpp::get_logger("AttackerSkillTactic"),
        "Recommended attacker ID from game_analysis: %d (score: %.2f)",
        game_analysis.recommended_attacker_id, game_analysis.attacker_suitability_score);
      last_logged_id = game_analysis.recommended_attacker_id;
    }

    return [wm, game_analysis](const std::shared_ptr<RobotInfo> & robot) {
      // game_analysisで推奨ロボットが設定されている場合、そのロボットを最優先
      if (
        game_analysis.recommended_attacker_id >= 0 &&
        robot->id == static_cast<uint8_t>(game_analysis.recommended_attacker_id)) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("AttackerSkillTactic"),
          "Robot %d matches recommended attacker, returning cost 0.0", robot->id);
        return 0.0;  // 最高の適性（コスト最小）
      }

      // それ以外はボール距離ベース
      double distance = robot->getDistance(wm->ball().pos);
      RCLCPP_DEBUG(
        rclcpp::get_logger("AttackerSkillTactic"),
        "Robot %d cost: %.2f (ball distance)", robot->id, distance);
      return distance;
    };
  }

  bool isHardConstraint() const override
  {
    // game_analysisでrecommended_attacker_idが設定されている場合はハード制約として扱う
    const auto & game_analysis = getGameAnalysis();
    return game_analysis.recommended_attacker_id >= 0;
  }

protected:
  void onRobotsChanged() override { skill.reset(); }
};

}  // namespace crane
#endif  // CRANE_TACTICS__ATTACKER_SKILL_TACTIC_HPP_
