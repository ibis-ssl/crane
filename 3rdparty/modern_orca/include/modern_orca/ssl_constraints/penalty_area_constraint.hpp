// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <crane_basics/geometry_operations.hpp>

#include "ssl_constraint_base.hpp"

namespace crane::modern_orca
{

template <Agent AgentType>
class PenaltyAreaAvoidanceConstraint : public SSLConstraintBase<AgentType>
{
public:
  explicit PenaltyAreaAvoidanceConstraint(int priority = 90)
  : priority_(priority), penalty_area_offset_(0.1), surrounding_offset_(0.3)
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

    // 自チームと相手チームの両方のペナルティエリアをチェック
    generatePenaltyAreaConstraints(constraints, agent_pos, agent_radius, true);  // 自チームのペナルティエリア
    generatePenaltyAreaConstraints(
      constraints, agent_pos, agent_radius, false);  // 相手チームのペナルティエリア

    return constraints;
  }

  void updateFromWorldModel(const crane::WorldModelWrapper::SharedPtr & world_model) override
  {
    world_model_ = world_model;
  }

  void updateFromRefereeCommand(
    const robocup_ssl_msgs::msg::Referee::_command_type & command) override
  {
    // レフェリーコマンドに基づいてペナルティエリアオフセットを調整
    switch (command) {
      case robocup_ssl_msgs::msg::Referee::COMMAND_STOP:
      case robocup_ssl_msgs::msg::Referee::COMMAND_DIRECT_FREE_BLUE:
      case robocup_ssl_msgs::msg::Referee::COMMAND_DIRECT_FREE_YELLOW:
        penalty_area_offset_ = 0.5;
        surrounding_offset_ = 0.6;
        break;
      default:
        penalty_area_offset_ = 0.1;
        surrounding_offset_ = 0.3;
        break;
    }
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "PenaltyAreaAvoidanceConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    auto cloned = std::make_unique<PenaltyAreaAvoidanceConstraint>(*this);
    cloned->world_model_ = world_model_;
    return cloned;
  }

  SSLConstraintType getConstraintType() const noexcept override
  {
    return SSLConstraintType::PENALTY_AREA_AVOIDANCE;
  }

  // ペナルティエリア固有の設定
  void setPenaltyAreaOffset(double offset) { penalty_area_offset_ = offset; }
  void setSurroundingOffset(double offset) { surrounding_offset_ = offset; }
  double getPenaltyAreaOffset() const { return penalty_area_offset_; }
  double getSurroundingOffset() const { return surrounding_offset_; }

protected:
  bool isConstraintActive() const noexcept override { return world_model_ != nullptr; }

private:
  void generatePenaltyAreaConstraints(
    std::vector<HalfPlane> & constraints, const Vector2 & agent_pos, double agent_radius,
    bool is_our_area) const
  {
    auto penalty_area =
      is_our_area ? world_model_->getOurPenaltyArea() : world_model_->getTheirPenaltyArea();
    auto goal_center =
      is_our_area ? world_model_->getOurGoalCenter() : world_model_->getTheirGoalCenter();

    Point goal_pos(goal_center.x(), goal_center.y());

    // ペナルティエリアを制約生成用のポイントに変換
    const auto penalty_area_size = world_model_->penaltyAreaSize();

    // ペナルティエリアの境界を計算
    double area_min_x = goal_pos.x() - std::copysign(penalty_area_size.x(), goal_pos.x());
    double area_max_x = goal_pos.x();
    double area_min_y = goal_pos.y() - penalty_area_size.y() * 0.5;
    double area_max_y = goal_pos.y() + penalty_area_size.y() * 0.5;

    // オフセットとエージェント半径で拡張
    double total_offset = penalty_area_offset_ + agent_radius;

    // ペナルティエリアの各側面に対して制約を生成

    // フロント側（フィールド側）
    if (std::abs(agent_pos.x() - area_min_x) < total_offset) {
      Vector2 normal = Vector2(std::copysign(-1.0, goal_pos.x()), 0.0);
      Vector2d point =
        Vector2d(area_min_x - std::copysign(total_offset, goal_pos.x()), agent_pos.y());
      constraints.emplace_back(normal, point);
    }

    // 左側
    if (
      agent_pos.y() > area_min_y && agent_pos.y() < area_max_y &&
      std::abs(agent_pos.y() - area_min_y) < total_offset) {
      Vector2d normal = Vector2d(0.0, -1.0);
      Vector2d point = Vector2d(agent_pos.x(), area_min_y - total_offset);
      constraints.emplace_back(normal, point);
    }

    // 右側
    if (
      agent_pos.y() > area_min_y && agent_pos.y() < area_max_y &&
      std::abs(agent_pos.y() - area_max_y) < total_offset) {
      Vector2d normal = Vector2d(0.0, 1.0);
      Vector2d point = Vector2d(agent_pos.x(), area_max_y + total_offset);
      constraints.emplace_back(normal, point);
    }
  }

  crane::WorldModelWrapper::SharedPtr world_model_;
  int priority_;
  double penalty_area_offset_;
  double surrounding_offset_;
};

}  // namespace crane::modern_orca
