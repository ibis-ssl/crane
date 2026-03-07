// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/velocity_plan_tracker.hpp"

#include <cmath>
#include <crane_geometry/geometry_operations.hpp>

namespace crane
{

void VelocityPlanTracker::addPlanPoint(
  crane_msgs::msg::VelocityPlanTrace & trace, const std::string & source,
  const Eigen::Vector2d & predicted_pos, const Eigen::Vector2d & predicted_vel,
  int32_t target_time_us, int32_t estimated_arrival_time_us)
{
  crane_msgs::msg::VelocityPlanPoint point;

  point.source = source;
  point.target_time_us = target_time_us;
  point.predicted_pos_x = static_cast<float>(predicted_pos.x());
  point.predicted_pos_y = static_cast<float>(predicted_pos.y());
  point.predicted_vel_x = static_cast<float>(predicted_vel.x());
  point.predicted_vel_y = static_cast<float>(predicted_vel.y());
  point.estimated_arrival_time_us = estimated_arrival_time_us;

  trace.plan_points.push_back(point);
}

void VelocityPlanTracker::addCorrection(
  crane_msgs::msg::VelocityPlanTrace & trace, const std::string & source,
  const Eigen::Vector2d & before_vel, const Eigen::Vector2d & after_vel)
{
  crane_msgs::msg::VelocityCorrection correction;

  correction.source = source;
  correction.before_vel_x = static_cast<float>(before_vel.x());
  correction.before_vel_y = static_cast<float>(before_vel.y());
  correction.after_vel_x = static_cast<float>(after_vel.x());
  correction.after_vel_y = static_cast<float>(after_vel.y());

  // 速度変化量を計算
  Eigen::Vector2d delta = after_vel - before_vel;
  correction.velocity_delta = static_cast<float>(delta.norm());

  // 方向変化を計算（度）
  double before_angle = std::atan2(before_vel.y(), before_vel.x());
  double after_angle = std::atan2(after_vel.y(), after_vel.x());
  double angle_diff = normalizeAngle(after_angle - before_angle);

  correction.direction_delta_deg = static_cast<float>(angle_diff * 180.0 / M_PI);

  trace.corrections.push_back(correction);
}

void VelocityPlanTracker::recordActual(
  crane_msgs::msg::VelocityPlanTrace & trace, const Eigen::Vector2d & actual_pos,
  const Eigen::Vector2d & actual_vel)
{
  // 最新の計画点がない場合は何もしない
  if (trace.plan_points.empty()) {
    return;
  }

  // 最新の計画点を取得
  const auto & latest_plan = trace.plan_points.back();

  crane_msgs::msg::VelocityPlanActual actual;

  actual.plan_time_us = latest_plan.target_time_us;
  actual.planned_vel_x = latest_plan.predicted_vel_x;
  actual.planned_vel_y = latest_plan.predicted_vel_y;
  actual.planned_pos_x = latest_plan.predicted_pos_x;
  actual.planned_pos_y = latest_plan.predicted_pos_y;

  actual.actual_vel_x = static_cast<float>(actual_vel.x());
  actual.actual_vel_y = static_cast<float>(actual_vel.y());
  actual.actual_pos_x = static_cast<float>(actual_pos.x());
  actual.actual_pos_y = static_cast<float>(actual_pos.y());

  // 速度誤差を計算
  Eigen::Vector2d planned_vel(latest_plan.predicted_vel_x, latest_plan.predicted_vel_y);
  Eigen::Vector2d vel_error = actual_vel - planned_vel;
  actual.velocity_error = static_cast<float>(vel_error.norm());

  // 位置誤差を計算
  Eigen::Vector2d planned_pos(latest_plan.predicted_pos_x, latest_plan.predicted_pos_y);
  Eigen::Vector2d pos_error = actual_pos - planned_pos;
  actual.position_error = static_cast<float>(pos_error.norm());

  // リングバッファとして動作（最大10件まで）
  trace.actuals.push_back(actual);
  if (trace.actuals.size() > MAX_ACTUALS) {
    trace.actuals.erase(trace.actuals.begin());
  }
}

}  // namespace crane
