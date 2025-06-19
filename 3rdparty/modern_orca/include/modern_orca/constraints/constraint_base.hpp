// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "../concepts.hpp"
#include "../types.hpp"

namespace crane::modern_orca
{

template <Agent AgentType>
class ConstraintBase
{
public:
  using agent_type = AgentType;

  virtual ~ConstraintBase() = default;

  virtual auto generateHalfPlanes(const AgentType & agent, double dt) const
    -> std::vector<HalfPlane> = 0;

  virtual auto priority() const noexcept -> int { return 0; }
  virtual auto isActive() const noexcept -> bool { return true; }
  virtual auto name() const -> std::string = 0;

  virtual auto clone() const -> std::unique_ptr<ConstraintBase> = 0;

protected:
  constexpr ConstraintBase() = default;
};

template <Agent AgentType>
class HalfPlaneConstraint : public ConstraintBase<AgentType>
{
public:
  HalfPlaneConstraint(const Vector2 & normal, const Vector2 & point, int priority = 0)
  : normal_(normal.normalized()), point_(point), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & /*agent*/, double /*dt*/) const
    -> std::vector<HalfPlane> override
  {
    return {HalfPlane{normal_, point_}};
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "HalfPlaneConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    return std::make_unique<HalfPlaneConstraint>(*this);
  }

  auto normal() const noexcept -> const Vector2 & { return normal_; }
  auto point() const noexcept -> const Vector2 & { return point_; }

  void setNormal(const Vector2 & normal) { normal_ = normal.normalized(); }
  void setPoint(const Vector2d & point) { point_ = point; }

private:
  Vector2d normal_;
  Vector2d point_;
  int priority_;
};

template <Agent AgentType>
class BoundaryConstraint : public ConstraintBase<AgentType>
{
public:
  BoundaryConstraint(
    const Vector2d & min_bounds, const Vector2d & max_bounds, double margin = 0.1,
    int priority = 100)
  : min_bounds_(min_bounds), max_bounds_(max_bounds), margin_(margin), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, double /*dt*/) const
    -> std::vector<HalfPlane> override
  {
    std::vector<HalfPlane> constraints;
    const auto pos = agent.position();
    const auto radius = agent.radius();

    if (pos.x() - radius < min_bounds_.x() + margin_) {
      constraints.emplace_back(
        Vector2d{1.0, 0.0}, Vector2d{min_bounds_.x() + margin_ + radius, pos.y()});
    }

    if (pos.x() + radius > max_bounds_.x() - margin_) {
      constraints.emplace_back(
        Vector2d{-1.0, 0.0}, Vector2d{max_bounds_.x() - margin_ - radius, pos.y()});
    }

    if (pos.y() - radius < min_bounds_.y() + margin_) {
      constraints.emplace_back(
        Vector2d{0.0, 1.0}, Vector2d{pos.x(), min_bounds_.y() + margin_ + radius});
    }

    if (pos.y() + radius > max_bounds_.y() - margin_) {
      constraints.emplace_back(
        Vector2d{0.0, -1.0}, Vector2d{pos.x(), max_bounds_.y() - margin_ - radius});
    }

    return constraints;
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "BoundaryConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    return std::make_unique<BoundaryConstraint>(*this);
  }

private:
  Vector2d min_bounds_;
  Vector2d max_bounds_;
  double margin_;
  int priority_;
};

template <Agent AgentType>
class SpeedLimitConstraint : public ConstraintBase<AgentType>
{
public:
  explicit SpeedLimitConstraint(double max_speed, int priority = 50)
  : max_speed_(max_speed), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, double /*dt*/) const
    -> std::vector<HalfPlane> override
  {
    std::vector<HalfPlane> constraints;

    const auto current_speed = agent.velocity().norm();
    if (current_speed > max_speed_ + EPSILON) {
      const auto vel_normalized = agent.velocity().normalized();
      constraints.emplace_back(-vel_normalized, agent.position() + max_speed_ * vel_normalized);
    }

    return constraints;
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "SpeedLimitConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    return std::make_unique<SpeedLimitConstraint>(*this);
  }

private:
  double max_speed_;
  int priority_;
};

template <Agent AgentType>
class CircularObstacleConstraint : public ConstraintBase<AgentType>
{
public:
  CircularObstacleConstraint(
    const Vector2d & center, double radius, double margin = 0.1, int priority = 75)
  : center_(center), radius_(radius), margin_(margin), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, double /*dt*/) const
    -> std::vector<HalfPlane> override
  {
    std::vector<HalfPlane> constraints;

    const auto agent_pos = agent.position();
    const auto agent_radius = agent.radius();
    const auto total_radius = radius_ + agent_radius + margin_;

    const auto relative_pos = agent_pos - center_;
    const auto distance = relative_pos.norm();

    if (distance < total_radius + EPSILON) {
      Vector2d normal = relative_pos.normalized();
      if (isZero(normal)) {
        normal = Vector2d{1.0, 0.0};
      }

      const auto constraint_point = center_ + total_radius * normal;
      constraints.emplace_back(normal, constraint_point);
    }

    return constraints;
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "CircularObstacleConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    return std::make_unique<CircularObstacleConstraint>(*this);
  }

private:
  Vector2d center_;
  double radius_;
  double margin_;
  int priority_;
};

}  // namespace crane::modern_orca
