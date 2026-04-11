// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msgs/msg/ball_info.hpp>
#include <crane_msgs/msg/practice_mode.hpp>
#include <crane_msgs/msg/world_model.hpp>
#include <crane_physics/ball_info.hpp>
#include <crane_physics/robot_info.hpp>
#include <crane_physics/slack_time_config.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "delay_monitor_wrapper.hpp"
#include "play_situation_wrapper.hpp"
#include "point_checker.hpp"
#include "robots_query.hpp"

namespace crane
{
using RobotList = std::vector<std::shared_ptr<RobotInfo>>;

// Forward declaration for PImpl
class BallOwnerCalculator;

// ボール所有者計算の結果を表す構造体
struct BallOwnerScore
{
  std::shared_ptr<RobotInfo> robot = nullptr;
  double min_slack = 100.;
  double min_slack_pos_distance = 100.;
  double max_slack = -100.;
  double score = 0.0;
};

struct TeamInfo
{
  Box penalty_area;

  RobotList robots;

  uint32_t max_allowed_bots;

  uint8_t goalie_id;

  /**
   * @brief ロボットリストをクエリ形式で絞り込むためのFluent Builder APIを開始
   * @return RobotsQueryインスタンス
   *
   * 使用例:
   * @code
   * auto robots = team.robotsWhere().available().excludeGoalie().get();
   * @endcode
   */
  [[nodiscard]] auto robotsWhere() const -> RobotsQuery { return RobotsQuery(robots, goalie_id); }
};

struct WorldModelWrapper : public DelayMonitorMixin<WorldModelWrapper>
{
  friend class DelayMonitorMixin<WorldModelWrapper>;
  using SharedPtr = std::shared_ptr<WorldModelWrapper>;

  using UniquePtr = std::unique_ptr<WorldModelWrapper>;

  explicit WorldModelWrapper(rclcpp::Node & node, bool setup_subscriber = true);

  ~WorldModelWrapper();

  auto update(const crane_msgs::msg::WorldModel & world_model) -> void;

  auto update(const crane_msgs::msg::GameAnalysis & msg) -> void { latest_msg.game_analysis = msg; }

  auto overwriteBallPos(Point pos, double z = 0.0) -> void
  {
    ball_.pos = pos;
    ball_.pos_z = z;
    // Ball構造体からBallInfoメッセージに同期
    ball_.toMsg(latest_msg.ball_info);
  }

  [[nodiscard]] const auto & getMsg() const { return latest_msg; }

  auto & getEditableMsg() { return latest_msg; }

  [[nodiscard]] auto onPositiveHalf() const { return (latest_msg.on_positive_half); }

  [[nodiscard]] auto getOurSideSign() const { return onPositiveHalf() ? 1.0 : -1.0; }

  [[nodiscard]] auto isYellow() const { return (latest_msg.is_yellow); }

  [[nodiscard]] auto hasUpdated() const { return has_updated; }

  [[nodiscard]] auto isEmplacePositiveSide() const { return latest_msg.is_emplace_positive_side; }

  // === 半面練習モード ===
  [[nodiscard]] auto practiceMode() const -> const crane_msgs::msg::PracticeMode &
  {
    return latest_msg.practice_mode;
  }
  [[nodiscard]] auto isPracticeModeEnabled() const { return latest_msg.practice_mode.enabled; }
  [[nodiscard]] auto isPracticeKickProhibited() const
  {
    return latest_msg.practice_mode.enabled && latest_msg.practice_mode.kick_prohibition;
  }
  [[nodiscard]] auto isPracticeKickProhibitedFor(const std::string & skill_name) const
  {
    if (!isPracticeKickProhibited()) {
      return false;
    }
    const auto & allowed = latest_msg.practice_mode.kick_allowed_skills;
    return std::find(allowed.begin(), allowed.end(), skill_name) == allowed.end();
  }
  [[nodiscard]] auto isPracticeReverseAttack() const
  {
    return latest_msg.practice_mode.enabled && latest_msg.practice_mode.reverse_attack;
  }

  /// @brief 攻撃対象ゴール中心を取得（練習モード reverse_attack 時は自陣ゴール）
  /// @note DF/ゴーリーはこのメソッドを使わず getOurGoalCenter() を直接使うこと
  [[nodiscard]] auto getAttackGoalCenter() const -> Point
  {
    return isPracticeReverseAttack() ? getOurGoalCenter() : getTheirGoalCenter();
  }

  /// @brief 攻撃対象ゴールポストを取得（練習モード reverse_attack 時は自陣ゴール）
  [[nodiscard]] auto getAttackGoalPosts() const -> std::pair<Point, Point>
  {
    return isPracticeReverseAttack() ? getOurGoalPosts() : getTheirGoalPosts();
  }

  /// @brief 攻撃対象ゴールラインを取得（練習モード reverse_attack 時は自陣ゴール）
  [[nodiscard]] auto getAttackGoalLine() const -> Segment
  {
    return isPracticeReverseAttack() ? getOurGoalLine() : getTheirGoalLine();
  }

  /// @brief 攻撃方向の符号を取得（練習モード reverse_attack 時は反転）
  /// @note FW/アタッカー系のみ使用。DF/ゴーリーは getOurSideSign() を使うこと
  [[nodiscard]] auto getAttackSideSign() const
  {
    return isPracticeReverseAttack() ? -getOurSideSign() : getOurSideSign();
  }

  auto addCallback(std::function<void(void)> && callback_func) -> void
  {
    callbacks.emplace_back(callback_func);
  }

  [[nodiscard]] auto getRobot(RobotIdentifier robot) const
  {
    if (robot.is_ours) {
      return ours_.robots.at(robot.id);
    } else {
      return theirs_.robots.at(robot.id);
    }
  }

  [[nodiscard]] auto getOurRobot(uint8_t id) const { return ours_.robots.at(id); }

  [[nodiscard]] auto getTheirRobot(uint8_t id) const { return theirs_.robots.at(id); }

  [[nodiscard]] auto getOurMaxAllowedBots() const { return ours_.max_allowed_bots; }

  [[nodiscard]] auto getTheirMaxAllowedBots() const { return theirs_.max_allowed_bots; }

  [[nodiscard]] auto generateFieldPoints(float grid_size) const;

  struct RobotWithDistance
  {
    RobotInfo::SharedPtr robot;
    double distance;
  };
  [[nodiscard]] auto getNearestRobotWithDistanceFromPoint(
    const Point & point, const RobotList & robots) const -> std::optional<RobotWithDistance>;

  [[nodiscard]] auto getNearestRobotWithDistanceFromSegment(
    const Segment & segment, const RobotList & robots) const -> std::optional<RobotWithDistance>;

  /// @brief 指定線分から最も近い敵ロボットを取得
  /// @param segment 基準線分
  /// @return 最も近い敵ロボットとその距離
  [[nodiscard]] auto getNearestTheirRobotFromSegment(const Segment & segment) const
    -> std::optional<RobotWithDistance>
  {
    return getNearestRobotWithDistanceFromSegment(segment, theirs_.robotsWhere().available().get());
  }

  [[nodiscard]] auto getDefenseWidth() const
  {
    return ours_.penalty_area.max_corner().y() - ours_.penalty_area.min_corner().y();
  }

  [[nodiscard]] auto getDefenseHeight() const
  {
    return ours_.penalty_area.max_corner().x() - ours_.penalty_area.min_corner().x();
  }

  [[nodiscard]] auto getOurGoalPosts() const -> std::pair<Point, Point>
  {
    double x = getOurGoalCenter().x();
    return {Point(x, latest_msg.goal_size.y * 0.5), Point(x, -latest_msg.goal_size.y * 0.5)};
  }

  [[nodiscard]] auto getTheirGoalPosts() const -> std::pair<Point, Point>
  {
    double x = getTheirGoalCenter().x();
    return {Point(x, latest_msg.goal_size.y * 0.5), Point(x, -latest_msg.goal_size.y * 0.5)};
  }

  [[nodiscard]] auto getOurPenaltyArea() const { return ours_.penalty_area; }

  [[nodiscard]] auto getTheirPenaltyArea() const { return theirs_.penalty_area; }

  [[nodiscard]] auto getOurGoalCenter() const -> Point { return goal_; }

  [[nodiscard]] auto getTheirGoalCenter() const -> Point { return Point(-goal_.x(), goal_.y()); }

  // === 便利関数: ゴールライン関連 ===
  /// @brief 自ゴールライン（ポスト間の線分）を取得
  /// @return 自ゴールの左右ポスト間の線分
  [[nodiscard]] auto getOurGoalLine() const -> Segment
  {
    auto [post1, post2] = getOurGoalPosts();
    return {post1, post2};
  }

  /// @brief 敵ゴールライン（ポスト間の線分）を取得
  /// @return 敵ゴールの左右ポスト間の線分
  [[nodiscard]] auto getTheirGoalLine() const -> Segment
  {
    auto [post1, post2] = getTheirGoalPosts();
    return {post1, post2};
  }

  /// @brief ボールが自ゴールに向かっているかを判定
  /// @return ボール軌道が自ゴールラインと交差する場合true
  [[nodiscard]] auto isShootingTowardsOurGoal() const -> bool;

  /// @brief ボールが敵ゴールに向かっているかを判定
  /// @return ボール軌道が敵ゴールラインと交差する場合true
  [[nodiscard]] auto isShootingTowardsTheirGoal() const -> bool;

  [[nodiscard]] auto getBallPlacementTarget() const -> std::optional<Point>;

  // rule 8.4.3
  [[nodiscard]] auto getBallPlacementArea(double offset = 0.) const -> std::optional<Capsule>;

  [[nodiscard]] auto getOurGoalieId() const { return ours_.goalie_id; }

  [[nodiscard]] auto getTheirGoalieId() const { return theirs_.goalie_id; }

  /**
   *
   * @param from
   * @return {angle, width}
   */
  struct GoalAngleRange
  {
    double center_angle;
    double angle_width;
  };
  [[nodiscard]] auto getLargestGoalAngleRangeFromPoint(
    const Point from, const std::pair<Point, Point> & goal_posts,
    const RobotList & obstacle_robots) const -> GoalAngleRange;

  [[nodiscard]] auto getLargestGoalAngleRangeFromPoint(const Point from) const -> GoalAngleRange
  {
    return getLargestGoalAngleRangeFromPoint(
      from, getTheirGoalPosts(), theirs_.robotsWhere().available().get());
  }

  [[nodiscard]] auto getLargestOurGoalAngleRangeFromPoint(const Point from) const -> GoalAngleRange
  {
    return getLargestGoalAngleRangeFromPoint(
      from, getOurGoalPosts(), ours_.robotsWhere().available().get());
  }

  /// @brief 攻撃方向を考慮したゴール角度範囲を取得（練習モード reverse_attack 対応）
  [[nodiscard]] auto getLargestAttackGoalAngleRangeFromPoint(const Point from) const
    -> GoalAngleRange
  {
    if (isPracticeReverseAttack()) {
      return getLargestGoalAngleRangeFromPoint(
        from, getOurGoalPosts(), ours_.robotsWhere().available().get());
    }
    return getLargestGoalAngleRangeFromPoint(
      from, getTheirGoalPosts(), theirs_.robotsWhere().available().get());
  }

  struct SlackTimeResult
  {
    double slack_time;

    Point intercept_point;

    std::shared_ptr<RobotInfo> robot;
  };

  [[nodiscard]] auto getBallSequence(double t_horizon, double t_step)
    -> std::vector<std::pair<Point, double>>;

  [[nodiscard]] auto getBallSlackTime(
    const Point & ball_origin, const Vector2 & ball_velocity, double time, const RobotList & robots,
    const SlackTimeConfig & config) -> std::optional<SlackTimeResult>;

  /// @brief 事前計算済みのインターセプト地点からスラックタイムを計算するオーバーロード
  /// @details 減速モデルで生成したボールシーケンスと組み合わせて使用する。
  ///          ボール位置の再計算を行わず、robot到達時間のみを計算する。
  [[nodiscard]] auto getBallSlackTime(
    const Point & intercept_point, double time, const RobotList & robots,
    const SlackTimeConfig & config) -> std::optional<SlackTimeResult>;

  [[nodiscard]] auto getBallSlackTime(
    double time, const RobotList & robots, const SlackTimeConfig & config)
    -> std::optional<SlackTimeResult>;

  [[nodiscard]] auto getSlackInterceptPointAndSlackTimeArray(
    const Point & ball_origin, const Vector2 & ball_velocity, const RobotList & robots,
    const SlackTimeConfig & config) -> std::vector<SlackTimeResult>;

  [[nodiscard]] auto getSlackInterceptPointAndSlackTimeArray(
    const RobotList & robots, const SlackTimeConfig & config) -> std::vector<SlackTimeResult>;

  [[nodiscard]] auto getMinMaxSlackInterceptPointAndSlackTime(
    const RobotList & robots, const SlackTimeConfig & config)
    -> std::pair<std::optional<SlackTimeResult>, std::optional<SlackTimeResult>>;

  [[nodiscard]] auto getBallSlackTime(double time, const RobotList & robots)
    -> std::optional<SlackTimeResult>
  {
    return getBallSlackTime(time, robots, slack_config_);
  }

  [[nodiscard]] auto getSlackInterceptPointAndSlackTimeArray(const RobotList & robots)
    -> std::vector<SlackTimeResult>
  {
    return getSlackInterceptPointAndSlackTimeArray(robots, slack_config_);
  }

  [[nodiscard]] auto getMinMaxSlackInterceptPointAndSlackTime(const RobotList & robots)
    -> std::pair<std::optional<SlackTimeResult>, std::optional<SlackTimeResult>>
  {
    return getMinMaxSlackInterceptPointAndSlackTime(robots, slack_config_);
  }

  // slack時間計算設定の取得・設定
  [[nodiscard]] auto getSlackConfig() const -> const SlackTimeConfig & { return slack_config_; }
  auto setSlackConfig(const SlackTimeConfig & config) -> void { slack_config_ = config; }

  // Getter methods for accessing member variables
  [[nodiscard]] auto ours() const -> const TeamInfo & { return ours_; }
  [[nodiscard]] auto theirs() const -> const TeamInfo & { return theirs_; }
  [[nodiscard]] auto ball() const -> const Ball & { return ball_; }
  [[nodiscard]] auto fieldSize() const -> const Point & { return field_size_; }
  [[nodiscard]] auto penaltyAreaSize() const -> const Point & { return penalty_area_size_; }
  [[nodiscard]] auto goalSize() const -> const Point & { return goal_size_; }
  [[nodiscard]] auto goal() const -> const Point & { return goal_; }

private:
  std::unique_ptr<BallOwnerCalculator> ball_owner_calculator_;
  bool ball_owner_calculator_enabled_ = false;

public:
  auto setBallOwnerCalculatorEnabled(bool enabled = true) -> void;

  [[nodiscard]] auto getOurFrontier() const -> std::optional<BallOwnerScore>;

  [[nodiscard]] auto getTheirFrontier() const -> std::optional<BallOwnerScore>;

  auto getPenaltyAreaCorners(double offset_x, double offset_y) const
    -> std::tuple<Point, Point, Point, Point>;
  auto getOurAreaCorners() const -> std::tuple<Point, Point, Point, Point>;

  auto getIntersectionOurPenaltyArea(
    const Segment & target_segment, double offset_x, double offset_y) const -> std::optional<Point>;

  auto getForwardDefenseRatio(const Segment & ball_line) const -> std::optional<double>;

  PointChecker point_checker;

  // ===== 遅延監視関連メソッド（DelayMonitorMixin経由） =====

  auto getDelayCheckpoints() -> crane_msgs::msg::DelayCheckpoints &
  {
    return latest_msg.delay_checkpoints;
  }

  [[nodiscard]] auto getDelayCheckpoints() const -> const crane_msgs::msg::DelayCheckpoints &
  {
    return latest_msg.delay_checkpoints;
  }

private:
  rclcpp::Subscription<crane_msgs::msg::WorldModel>::SharedPtr subscriber;

  std::vector<std::function<void(void)>> callbacks;

  crane_msgs::msg::WorldModel latest_msg;

  bool has_updated = false;

  TeamInfo ours_;
  TeamInfo theirs_;
  Point field_size_;
  Point penalty_area_size_;
  Point goal_size_;
  Point goal_;
  Ball ball_;

  // slack時間計算設定
  SlackTimeConfig slack_config_;

  // 診断情報の統合
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_agg_sub_;
  std::map<uint8_t, bool> robot_diagnostic_errors_;  // robot_id -> has_error

  auto diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) -> void;

  auto updateRobotTimestamps(
    RobotInfo & info, const crane_msgs::msg::RobotInfo & robot_msg, const rclcpp::Time & now)
    -> void;
};
}  // namespace crane

#endif  // CRANE_MSG_WRAPPERS__WORLD_MODEL_WRAPPER_HPP_
