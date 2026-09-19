// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__ATTACKER_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__ATTACKER_SKILL_SESSION_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/pass_plan.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/attacker.hpp>
#include <crane_sessions/session_base.hpp>
#include <functional>
#include <magic_enum/magic_enum.hpp>
#include <memory>
#include <optional>
#include <range/v3/algorithm/contains.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
class AttackerSkillSession : public SessionBase
{
  // アロケータのhysteresis_bonus(1.5m)を確実に上回り、game_analyzer推奨の切替を保証するマージン
  static constexpr double RECOMMENDED_ATTACKER_MARGIN = 2.0;

  /// 出し手がボールを持っていると見なす距離 [m]。
  static constexpr double PASS_BALL_OWNED_DISTANCE = 0.25;
  /// ボールが出し手から離れた（＝蹴った）と見なす距離 [m]。
  /// ドリブル中の揺れでは越えない値にする。
  static constexpr double PASS_BALL_RELEASED_DISTANCE = 0.6;
  /// 蹴ったあと出し手が追わずに待つ時間 [s]。
  /// 実測のパス到達時間は 0.8〜1.3 秒なので、受け手が確保するまで足りる長さにする。
  static constexpr double PASS_YIELD_DURATION = 2.0;

public:
  std::shared_ptr<skills::Attacker> skill = nullptr;

  COMPOSITION_PUBLIC explicit AttackerSkillSession(
    WorldModelWrapper::SharedPtr & world_model, [[maybe_unused]] rclcpp::Node & node)
  : SessionBase("attacker_skill", world_model)
  {
  }

  /// 自分が出したパスを自分で追いかけないようにするか。
  ///
  /// 飛行状態（STATE_BALL_IN_FLIGHT）は条件に使わない。ボールが飛んでいるかの
  /// 推定は現状安定しておらず、特に実機で安定しないため、そこに依存した判断は
  /// 実機で静かに無効化される（docs/pass.md の制約を参照）。
  ///
  /// 代わりに、このセッション自身が持っている事実だけで判断する。
  ///   1. 直前に「自分が出し手の計画」をボールを持った状態で保持していた
  ///   2. そのボールが自分から離れた
  /// どちらも位置の直接観測で、速度推定も飛行判定も挟まない。
  ///
  /// これが無いと、受領点を通過したボールが転がり続けたときに、ボールへ最も近い
  /// 出し手がそのまま追いかけて自分のパスを取り戻す。実測では SELF_TOUCH として
  /// 現れ、味方ゲート修正後に残る支配的な失敗だった。
  auto shouldYieldAfterOwnPass(uint8_t robot_id) -> bool
  {
    const auto & plan = world_model->getMsg().game_analysis.pass_plan;
    const auto robot = world_model->getOurRobot(robot_id);
    const double ball_distance = robot->getDistance(world_model->ball().pos);
    const auto now = world_model->getMsg().header.stamp;
    const double now_sec = now.sec + now.nanosec * 1e-9;

    if (
      isUsablePassPlan(plan, *world_model) && plan.kicker_id == static_cast<int>(robot_id) &&
      ball_distance < PASS_BALL_OWNED_DISTANCE) {
      // パスを出す直前。まだ蹴っていない。
      holding_pass_as_kicker_ = true;
      pass_issued_at_sec_.reset();
      return false;
    }

    if (holding_pass_as_kicker_ && ball_distance > PASS_BALL_RELEASED_DISTANCE) {
      holding_pass_as_kicker_ = false;
      pass_issued_at_sec_ = now_sec;
    }

    if (!pass_issued_at_sec_.has_value()) {
      return false;
    }
    if (now_sec - *pass_issued_at_sec_ > PASS_YIELD_DURATION) {
      pass_issued_at_sec_.reset();
      return false;
    }
    // 待っているあいだにボールが自分の所へ戻ってきたら、譲る理由が無い。
    if (ball_distance < PASS_BALL_OWNED_DISTANCE) {
      pass_issued_at_sec_.reset();
      return false;
    }
    return true;
  }

  /// 自分が出し手の計画をボールを持った状態で保持しているか。
  bool holding_pass_as_kicker_ = false;
  /// 自分がパスを出したと判断した時刻 [s]。譲っていない間は無効。
  std::optional<double> pass_issued_at_sec_;

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    // GlobalRobotAllocator対応: robotsが変更されたらスキルを再生成
    if (robots.empty()) {
      return {SessionBase::Status::RUNNING, {}};
    }
    if (not skill) {
      skill = std::make_shared<skills::Attacker>(robots.front().id, world_model);
      visualizer->layer = "skill/" + skill->name;
    }
    if (shouldYieldAfterOwnPass(robots.front().id)) {
      skill->commander()->stopHere().lookAtBall().kickStraight(0.0);
      return {SessionBase::Status::RUNNING, {skill->getRobotCommand()}};
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
    return {static_cast<SessionBase::Status>(status), {skill->getRobotCommand()}};
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    auto wm = world_model;                   // shared_ptrをコピー
    auto game_analysis = getGameAnalysis();  // GameAnalysisをコピー
    game_analysis.pass_plan = wm->getMsg().game_analysis.pass_plan;

    // デバッグ用：推奨ロボットIDをログ出力
    static int last_logged_id = -999;
    if (game_analysis.recommended_attacker_id != last_logged_id) {
      RCLCPP_INFO(
        rclcpp::get_logger("AttackerSkillSession"),
        "Recommended attacker ID from game_analysis: %d (score: %.2f)",
        game_analysis.recommended_attacker_id, game_analysis.attacker_suitability_score);
      last_logged_id = game_analysis.recommended_attacker_id;
    }

    return [wm, game_analysis](const std::shared_ptr<RobotInfo> & robot) {
      if (isUsablePassPlan(game_analysis.pass_plan, *wm)) {
        if (robot->id == game_analysis.pass_plan.receiver_id) {
          return 1000.0;
        }
        if (robot->id == game_analysis.pass_plan.kicker_id) {
          return 0.0;
        }
        return robot->getDistance(wm->ball().pos) + RECOMMENDED_ATTACKER_MARGIN;
      }
      // game_analysisで推奨ロボットが設定されている場合、そのロボットを最優先
      if (
        game_analysis.recommended_attacker_id >= 0 &&
        robot->id == static_cast<uint8_t>(game_analysis.recommended_attacker_id)) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("AttackerSkillSession"),
          "Robot %d matches recommended attacker, returning cost 0.0", robot->id);
        return 0.0;  // 最高の適性（コスト最小）
      }

      // それ以外はボール距離ベース
      double distance = robot->getDistance(wm->ball().pos);
      // game_analyzerのSelectionHysteresisが安定性を担保済みのため、
      // アロケータのhysteresis_bonus(1.5m)を確実に上回るマージンを付与して推奨切替を阻害しない
      double cost = distance + RECOMMENDED_ATTACKER_MARGIN;
      RCLCPP_DEBUG(
        rclcpp::get_logger("AttackerSkillSession"), "Robot %d cost: %.2f (ball distance + margin)",
        robot->id, cost);
      return cost;
    };
  }

protected:
  void onRobotsChanged() override { skill.reset(); }
};

}  // namespace crane
#endif  // CRANE_SESSIONS__ATTACKER_SKILL_SESSION_HPP_
