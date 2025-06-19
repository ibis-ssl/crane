// Copyright 2024 ibis-ssl
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "../concepts.hpp"
#include "../types.hpp"

namespace modern_orca
{

template <Agent AgentType>
class ConstraintBase
{
public:
  using agent_type = AgentType;

  virtual ~ConstraintBase() = default;

  virtual auto generateHalfPlanes(const AgentType & agent, TimeStep dt) const
    -> std::vector<HalfPlaneD> = 0;

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
  HalfPlaneConstraint(const Vector2D & normal, const Vector2D & point, int priority = 0)
  : normal_(normal.normalized()), point_(point), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & /*agent*/, TimeStep /*dt*/) const
    -> std::vector<HalfPlaneD> override
  {
    return {HalfPlaneD{normal_, point_}};
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "HalfPlaneConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    return std::make_unique<HalfPlaneConstraint>(*this);
  }

  auto normal() const noexcept -> const Vector2D & { return normal_; }
  auto point() const noexcept -> const Vector2D & { return point_; }

  void setNormal(const Vector2D & normal) { normal_ = normal.normalized(); }
  void setPoint(const Vector2D & point) { point_ = point; }

private:
  Vector2D normal_;
  Vector2D point_;
  int priority_;
};

template <Agent AgentType>
class BoundaryConstraint : public ConstraintBase<AgentType>
{
public:
  BoundaryConstraint(
    const Vector2D & min_bounds, const Vector2D & max_bounds, Scalar margin = 0.1,
    int priority = 100)
  : min_bounds_(min_bounds), max_bounds_(max_bounds), margin_(margin), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, TimeStep /*dt*/) const
    -> std::vector<HalfPlaneD> override
  {
    std::vector<HalfPlaneD> constraints;
    const auto pos = agent.position();
    const auto radius = agent.radius();

    if (pos.x() - radius < min_bounds_.x() + margin_) {
      constraints.emplace_back(
        Vector2D{1.0, 0.0}, Vector2D{min_bounds_.x() + margin_ + radius, pos.y()});
    }

    if (pos.x() + radius > max_bounds_.x() - margin_) {
      constraints.emplace_back(
        Vector2D{-1.0, 0.0}, Vector2D{max_bounds_.x() - margin_ - radius, pos.y()});
    }

    if (pos.y() - radius < min_bounds_.y() + margin_) {
      constraints.emplace_back(
        Vector2D{0.0, 1.0}, Vector2D{pos.x(), min_bounds_.y() + margin_ + radius});
    }

    if (pos.y() + radius > max_bounds_.y() - margin_) {
      constraints.emplace_back(
        Vector2D{0.0, -1.0}, Vector2D{pos.x(), max_bounds_.y() - margin_ - radius});
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
  Vector2D min_bounds_;
  Vector2D max_bounds_;
  Scalar margin_;
  int priority_;
};

template <Agent AgentType>
class SpeedLimitConstraint : public ConstraintBase<AgentType>
{
public:
  explicit SpeedLimitConstraint(Scalar max_speed, int priority = 50)
  : max_speed_(max_speed), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, TimeStep /*dt*/) const
    -> std::vector<HalfPlaneD> override
  {
    std::vector<HalfPlaneD> constraints;

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
  Scalar max_speed_;
  int priority_;
};

template <Agent AgentType>
class CircularObstacleConstraint : public ConstraintBase<AgentType>
{
public:
  CircularObstacleConstraint(
    const Vector2D & center, Scalar radius, Scalar margin = 0.1, int priority = 75)
  : center_(center), radius_(radius), margin_(margin), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, TimeStep /*dt*/) const
    -> std::vector<HalfPlaneD> override
  {
    std::vector<HalfPlaneD> constraints;

    const auto agent_pos = agent.position();
    const auto agent_radius = agent.radius();
    const auto total_radius = radius_ + agent_radius + margin_;

    const auto relative_pos = agent_pos - center_;
    const auto distance = relative_pos.norm();

    if (distance < total_radius + EPSILON) {
      Vector2D normal = relative_pos.normalized();
      if (normal.isZero()) {
        normal = Vector2D{1.0, 0.0};
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
  Vector2D center_;
  Scalar radius_;
  Scalar margin_;
  int priority_;
};

}  // namespace modern_orca
