// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__RVO2_PLANNER_HPP_
#define CRANE_LOCAL_PLANNER__RVO2_PLANNER_HPP_

#include <rvo2_vendor/RVO/RVO.h>

#include <crane_comm/parameter_with_event.hpp>
#include <crane_msg_wrappers/velocity_plan_tracker.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/msg/robot_commands.hpp>
#include <crane_msgs/msg/robot_feedback_array.hpp>
#include <crane_physics/pid_controller.hpp>
#include <memory>

#include "planner_base.hpp"

// cspell: ignore OBST
namespace crane
{

enum class ZeroVelocityReason {
  NONE,
  POSITION_TOLERANCE,
  RVO_COLLISION_OR_CONSTRAINT,
  PREF_VELOCITY_ZERO,
};

inline auto toString(ZeroVelocityReason reason) -> std::string
{
  switch (reason) {
    case ZeroVelocityReason::POSITION_TOLERANCE:
      return "POSITION_TOLERANCE";
    case ZeroVelocityReason::RVO_COLLISION_OR_CONSTRAINT:
      return "RVO_COLLISION_OR_CONSTRAINT";
    case ZeroVelocityReason::PREF_VELOCITY_ZERO:
      return "PREF_VELOCITY_ZERO";
    default:
      return "NONE";
  }
}

class RVO2Planner : public LocalPlannerBase
{
public:
  explicit RVO2Planner(rclcpp::Node & node);

  auto reflectWorldToRVOSim(crane_msgs::msg::RobotCommands & msg) -> void;

  auto extractVelocityCommandsFromRVOSim(
    const crane_msgs::msg::RobotCommands & msg, double theta_offset)
    -> crane_msgs::msg::RobotCommands;

  auto calculateRobotCommand(const crane_msgs::msg::RobotCommands & msg, double theta_offset)
    -> crane_msgs::msg::RobotCommands override;

  auto overrideTargetPosition(crane_msgs::msg::RobotCommands & msg) -> void;

  // セットプレイ中（INPLAY/HALT以外）かどうかを判定する。
  // STOP_PRE_*（値60-66）、FREE_KICK、BALL_PLACEMENT等が該当し、
  // これらの状態では敵ペナルティエリアへの離隔マージンを拡大する。
  static auto needsExpandedPenaltyAreaOffset(uint8_t cmd) -> bool
  {
    using PS = crane_msgs::msg::PlaySituation;
    switch (cmd) {
      case PS::INPLAY:
      case PS::OUR_INPLAY:
      case PS::THEIR_INPLAY:
      case PS::AMBIGUOUS_INPLAY:
      case PS::HALT:
      case PS::HALF_TIME:
      case PS::POST_GAME:
        return false;
      default:
        return true;
    }
  }

private:
  auto adjustForPenaltyAreaAvoidance(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::RobotCommand & command) const -> void;

  auto adjustForBallAvoidance(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::RobotCommand & command) const -> void;

  auto adjustForPlacementAvoidance(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::RobotCommand & command) const -> void;

  auto adjustForFieldBoundary(
    Point & target_pos, const Point & current_pos,
    const crane_msgs::msg::RobotCommand & command) const -> void;

  auto initializePenaltyAreaObstacles() -> void;

  std::unique_ptr<RVO::RVOSimulator> rvo_sim;

  crane_msgs::msg::RobotCommands pre_commands;

  auto toRVO(const Point & point) -> RVO::Vector2 { return RVO::Vector2(point.x(), point.y()); }

  auto toPoint(const RVO::Vector2 & vector) -> Point { return Point(vector.x(), vector.y()); }

  float RVO_TIME_STEP = 1.0 / 60.0f;
  float RVO_NEIGHBOR_DIST = 2.0f;
  int RVO_MAX_NEIGHBORS = 5;
  float RVO_TIME_HORIZON = 1.f;
  float RVO_TIME_HORIZON_OBST = 1.f;
  float RVO_RADIUS = 0.09f;
  float RVO_MAX_SPEED = 10.0f;

  double MAX_VEL = 5.0;
  double STOP_STATE_MAX_VELOCITY = 1.0;
  double FIELD_BOUNDARY_OFFSET = 0.2;

  // 衝突ファール (crashing) 回避パラメータ
  // SSLルール: 衝突時の速度差射影 > 1.5 m/s でファール
  double CRASH_SPEED_LIMIT = 1.5;
  double CRASH_SAFETY_MARGIN = 0.3;
  double CRASH_AVOIDANCE_DISTANCE = 1.0;
  double CRASH_AVOIDANCE_DECEL_DISTANCE = 0.5;

  // ペナルティエリア回避パラメータ
  double PENALTY_AREA_OFFSET = 0.1;       // ペナルティエリア判定マージン [m]（グローバル回避用）
  double PENALTY_AREA_OFFSET_STOP = 0.3;  // ペナルティエリア判定マージン [m]（STOP時）
  float PENALTY_AREA_TIME_HORIZON_OBST = 0.5f;  // RVO2障害物回避の時間ホライズン [s]

  bool penalty_area_obstacles_initialized = false;

  // 加速度は減速度の何倍にするかという係数
  ParameterWithEvent<double> acceleration_factor;

  rclcpp::Subscription<crane_msgs::msg::RobotFeedbackArray>::SharedPtr sub_feedback_array;

  crane_msgs::msg::RobotFeedbackArray latest_feedback;

  // 速度計画トレース有効化フラグ
  bool enable_velocity_plan_trace = false;
};
}  // namespace crane
#endif  // CRANE_LOCAL_PLANNER__RVO2_PLANNER_HPP_
