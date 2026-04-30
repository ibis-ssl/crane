// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__ROTATING_SINGLE_SKILL_SESSION_HPP_
#define CRANE_SESSIONS__ROTATING_SINGLE_SKILL_SESSION_HPP_

#include <crane_sessions/rotation_suitability.hpp>
#include <crane_sessions/session_base.hpp>
#include <crane_sessions/skill_assignment_history.hpp>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

namespace crane
{
/// 単一スキルにロボットを割り当て、履歴に基づきローテーションさせる Session の共通基底。
///
/// 派生クラス（例: BallPlacementSkillSession, FreeKickerSkillSession）が以下を実装する:
/// - createSkill(): スキルインスタンスを生成
/// - resolveHistoryFilePath(): 履歴ファイルパスを返す（スキルごとに別ファイル）
/// - determineAssignmentResult(): セッション終了時の成否判定
///
/// 共通化されているもの:
/// - スキル lifecycle（生成・active_robot_id 追跡・直近 Status 記録）
/// - 履歴ファイルの遅延ロードと成否記録
/// - getRobotSuitabilityFunc(): rotation_suitability.hpp の共通ロジックを使う
template <typename SkillType>
class RotatingSingleSkillSession : public SessionBase
{
public:
  std::shared_ptr<SkillType> skill = nullptr;

  using SessionBase::SessionBase;

  auto calculatePositionCommand(const std::vector<RobotIdentifier> & robots)
    -> std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> override
  {
    if (robots.empty()) {
      return {Status::RUNNING, {}};
    }
    if (not skill) {
      skill = createSkill(robots.front().id);
      visualizer->layer = "skill/" + skill->name;
      active_robot_id_ = robots.front().id;
      onSkillStarted();
    }
    onBeforeSkillRun();
    auto status = static_cast<Status>(skill->run());
    last_skill_status_ = status;
    return {status, {skill->getRobotCommand()}};
  }

  auto getRobotSuitabilityFunc() const
    -> std::function<double(const std::shared_ptr<RobotInfo> &)> override
  {
    ensureHistoryLoaded();

    RotationSuitabilityConfig config;
    config.success_weight =
      this->template getSessionParameter<double>("success_weight", config.success_weight);
    config.failure_weight =
      this->template getSessionParameter<double>("failure_weight", config.failure_weight);
    config.distance_weight =
      this->template getSessionParameter<double>("distance_weight", config.distance_weight);
    config.rotation_weight =
      this->template getSessionParameter<double>("rotation_weight", config.rotation_weight);
    config.top_n_candidates =
      this->template getSessionParameter<int>("top_n_candidates", config.top_n_candidates);

    return makeRotationSuitabilityFunc(this->world_model, history_, config);
  }

protected:
  /// スキルインスタンスを生成するファクトリメソッド（派生クラスで実装）。
  virtual auto createSkill(uint8_t robot_id) -> std::shared_ptr<SkillType> = 0;

  /// 履歴ファイルパスを解決する（派生クラスで実装、スキルごとに別ファイル）。
  virtual std::filesystem::path resolveHistoryFilePath() const = 0;

  /// セッションが Active 集合から外れた際の成否判定（派生クラスで実装）。
  /// - std::nullopt: 履歴更新なし（明示的シグナルなし）
  /// - true: 成功記録
  /// - false: 失敗記録
  ///
  /// 呼び出し時点で getActiveRobotId() / getLastSkillStatus() が参照可能。
  virtual std::optional<bool> determineAssignmentResult(
    const crane_msgs::msg::PlaySituation & current_play_situation) = 0;

  /// 初回 calculate 時にスキル生成直後に呼ばれる任意フック。
  /// 試行開始時点のスナップショット（例: ボールプレイスメントの failures カウント）を
  /// 派生クラスがキャプチャするのに使う。
  virtual void onSkillStarted() {}

  /// 各 calculate 呼び出しで skill->run() の直前に呼ばれる任意フック。
  /// 動的なスキルパラメータ更新（例: placement_x/y のセット）に使う。
  virtual void onBeforeSkillRun() {}

  /// onRobotsChanged() の派生クラス固有処理用フック。
  virtual void onRobotsChangedHook() {}

  /// 直近の skill->run() の Status（determineAssignmentResult から参照する用）。
  Status getLastSkillStatus() const { return last_skill_status_; }

  /// 現在割り当て中のロボットID（determineAssignmentResult から参照する用）。
  std::optional<uint8_t> getActiveRobotId() const { return active_robot_id_; }

  /// 履歴を遅延初期化する。
  void ensureHistoryLoaded() const
  {
    if (history_) {
      return;
    }
    history_ = std::make_shared<SkillAssignmentHistory>(resolveHistoryFilePath());
  }

  void onRobotsChanged() override
  {
    skill.reset();
    active_robot_id_.reset();
    last_skill_status_ = Status::RUNNING;
    onRobotsChangedHook();
  }

  void onDeactivated(const crane_msgs::msg::PlaySituation & current_play_situation) override
  {
    if (!active_robot_id_.has_value()) {
      return;
    }
    const auto robot_id = *active_robot_id_;

    // determineAssignmentResult は active_robot_id_ / last_skill_status_ を参照しうるため
    // リセットより前に呼ぶ。派生クラスもこの順序を前提に実装すること。
    const auto result = determineAssignmentResult(current_play_situation);

    active_robot_id_.reset();
    skill.reset();
    last_skill_status_ = Status::RUNNING;

    if (!result.has_value()) {
      RCLCPP_INFO(
        rclcpp::get_logger("RotatingSingleSkillSession"),
        "[%s] ロボット %d "
        "の試行終了を検知したが、明示的な成否シグナルがないため履歴更新をスキップ",
        this->name.c_str(), static_cast<int>(robot_id));
      return;
    }

    ensureHistoryLoaded();
    const bool success = *result;
    const bool saved =
      success ? history_->recordSuccess(robot_id) : history_->recordFailure(robot_id);
    const char * result_text = success ? "成功" : "失敗";
    if (saved) {
      RCLCPP_INFO(
        rclcpp::get_logger("RotatingSingleSkillSession"), "[%s] ロボット %d の%sを記録",
        this->name.c_str(), static_cast<int>(robot_id), result_text);
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("RotatingSingleSkillSession"), "[%s] ロボット %d の%s履歴の保存に失敗",
        this->name.c_str(), static_cast<int>(robot_id), result_text);
    }
  }

private:
  mutable std::shared_ptr<SkillAssignmentHistory> history_;
  std::optional<uint8_t> active_robot_id_;
  Status last_skill_status_ = Status::RUNNING;
};

}  // namespace crane
#endif  // CRANE_SESSIONS__ROTATING_SINGLE_SKILL_SESSION_HPP_
