// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <crane_geometry/geometry_operations.hpp>

#include "ssl_constraint_base.hpp"

namespace crane::modern_orca
{

template <Agent AgentType>
class BallAvoidanceConstraint : public SSLConstraintBase<AgentType>
{
public:
  explicit BallAvoidanceConstraint(int priority = 80) : priority_(priority), min_ball_distance_(0.2)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, double /*dt*/) const
    -> std::vector<HalfPlane> override
  {
    std::vector<HalfPlane> constraints;

    if (!world_model_ || !this->isEnabled()) {
      return constraints;
    }

    const auto agent_pos = agent.position();
    const auto agent_radius = agent.radius();
    const auto ball_pos = Point(world_model_->ball().pos.x(), world_model_->ball().pos.y());

    const auto distance_to_ball = (agent_pos - ball_pos).norm();
    const auto required_distance = min_ball_distance_ + agent_radius;

    // エージェントがボールに近すぎる場合、押し離す制約を作成
    if (distance_to_ball < required_distance) {
      Vector2 direction = agent_pos - ball_pos;
      if (direction.norm() < EPSILON) {
        direction = Vector2(1.0, 0.0);  // 重なっている場合のデフォルト方向
      } else {
        direction = direction.normalized();
      }

      // ボールからの最小距離を強制する半平面を作成
      const Vector2 constraint_point = ball_pos + direction * required_distance;
      constraints.emplace_back(direction, constraint_point);
    }

    return constraints;
  }

  void updateFromWorldModel(const crane::WorldModelWrapper::SharedPtr & world_model) override
  {
    world_model_ = world_model;
  }

  void updateFromRefereeCommand(
    const robocup_ssl_msgs::msg::Referee::_command_type & command) override
  {
    // レフェリーコマンドに基づいてボールの最小距離を更新
    switch (command) {
      case robocup_ssl_msgs::msg::Referee::COMMAND_DIRECT_FREE_BLUE:
      case robocup_ssl_msgs::msg::Referee::COMMAND_DIRECT_FREE_YELLOW:
        min_ball_distance_ = 0.7;
        break;
      case robocup_ssl_msgs::msg::Referee::COMMAND_STOP:
        min_ball_distance_ = 0.5;
        break;
      default:
        min_ball_distance_ = 0.2;
        break;
    }
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "BallAvoidanceConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    auto cloned = std::make_unique<BallAvoidanceConstraint>(*this);
    cloned->world_model_ = world_model_;
    return cloned;
  }

  SSLConstraintType getConstraintType() const noexcept override
  {
    return SSLConstraintType::BALL_AVOIDANCE;
  }

  // ボール固有の設定
  void setMinBallDistance(double distance) { min_ball_distance_ = distance; }
  double getMinBallDistance() const { return min_ball_distance_; }

protected:
  bool isConstraintActive() const noexcept override { return world_model_ != nullptr; }

private:
  crane::WorldModelWrapper::SharedPtr world_model_;
  int priority_;
  double min_ball_distance_;
};

}  // namespace crane::modern_orca
