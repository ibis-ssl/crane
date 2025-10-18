// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "ssl_constraint_base.hpp"

namespace crane::modern_orca
{

template <Agent AgentType>
class RefereeCommandConstraint : public SSLConstraintBase<AgentType>
{
public:
  explicit RefereeCommandConstraint(int priority = 70) : priority_(priority), max_speed_limit_(4.0)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, double /*dt*/) const
    -> std::vector<HalfPlane> override
  {
    std::vector<HalfPlane> constraints;

    if (!this->isEnabled()) {
      return constraints;
    }

    const auto agent_vel = agent.velocity();
    const auto current_speed = agent_vel.norm();

    // 現在の速度が制限を超える場合、速度制限制約を適用
    if (current_speed > max_speed_limit_ + EPSILON) {
      Vector2 vel_direction = agent_vel.normalized();

      // 現在の方向での速度を制限する制約を作成
      Vector2 constraint_point = agent.position() + vel_direction * max_speed_limit_;
      Vector2 constraint_normal = -vel_direction;

      constraints.emplace_back(constraint_normal, constraint_point);
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
    // レフェリーコマンドに基づいて速度制限を更新
    switch (command) {
      case robocup_ssl_msgs::msg::Referee::COMMAND_HALT:
        max_speed_limit_ = 0.0;
        break;
      case robocup_ssl_msgs::msg::Referee::COMMAND_STOP:
        max_speed_limit_ = 1.0;  // STOP中は1.0 m/s
        break;
      default:
        max_speed_limit_ = 4.0;
        break;
    }
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "RefereeCommandConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    auto cloned = std::make_unique<RefereeCommandConstraint>(*this);
    cloned->world_model_ = world_model_;
    return cloned;
  }

  SSLConstraintType getConstraintType() const noexcept override
  {
    return SSLConstraintType::REFEREE_COMMAND;
  }

  // レフェリーコマンド固有の設定
  void setMaxSpeedLimit(double speed) { max_speed_limit_ = speed; }
  double getMaxSpeedLimit() const { return max_speed_limit_; }

protected:
  bool isConstraintActive() const noexcept override
  {
    return true;  // 有効時は常にアクティブ
  }

private:
  crane::WorldModelWrapper::SharedPtr world_model_;
  int priority_;
  double max_speed_limit_;
};

}  // namespace crane::modern_orca
