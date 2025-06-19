// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <memory>
#include <vector>

#include "../agents/agent_base.hpp"
#include "constraint_base.hpp"

namespace crane::modern_orca
{

template <Agent AgentType>
class ORCAConstraint : public ConstraintBase<AgentType>
{
public:
  ORCAConstraint(
    const std::vector<AgentType *> & other_agents, double time_horizon = 2.0, int priority = 10)
  : other_agents_(other_agents), time_horizon_(time_horizon), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, double /*dt*/) const
    -> std::vector<HalfPlane> override
  {
    std::vector<HalfPlane> constraints;

    const auto agent_pos = agent.position();
    const auto agent_vel = agent.velocity();
    const auto agent_radius = agent.radius();

    for (const auto * other_agent : other_agents_) {
      if (other_agent->id() == agent.id()) continue;

      const auto other_pos = other_agent->position();
      const auto other_vel = other_agent->velocity();
      const auto other_radius = other_agent->radius();

      const auto relative_pos = other_pos - agent_pos;
      const auto relative_vel = agent_vel - other_vel;
      const auto combined_radius = agent_radius + other_radius;

      const auto distance_sq = relative_pos.squaredNorm();
      const auto combined_radius_sq = combined_radius * combined_radius;

      Vector2 line_direction;
      Vector2 line_point;

      if (distance_sq > combined_radius_sq) {
        const auto w = relative_vel - relative_pos / time_horizon_;
        const auto w_length_sq = w.squaredNorm();
        const auto dot_product = dot(w, relative_pos);

        if (dot_product < 0.0 && dot_product * dot_product > combined_radius_sq * w_length_sq) {
          const auto w_length = std::sqrt(w_length_sq);
          const auto unit_w = w / w_length;

          line_direction = Vector2{unit_w.y(), -unit_w.x()};
          const auto u = (combined_radius / time_horizon_ - w_length) * unit_w;
          line_point = agent_vel + 0.5 * u;
        } else {
          const auto distance = std::sqrt(distance_sq);
          const auto unit_w = relative_pos / distance;

          line_direction = Vector2{unit_w.y(), -unit_w.x()};
          const auto u = (combined_radius / time_horizon_ - distance) * unit_w;
          line_point = agent_vel + 0.5 * u;
        }
      } else {
        const auto time =
          (combined_radius * combined_radius - distance_sq) / (2.0 * combined_radius);
        const auto w = relative_vel - relative_pos / time;
        const auto w_length = w.norm();
        const auto unit_w = w / w_length;

        line_direction = Vector2{unit_w.y(), -unit_w.x()};
        const auto u = (combined_radius / time - w_length) * unit_w;
        line_point = agent_vel + 0.5 * u;
      }

      constraints.emplace_back(line_direction, line_point);
    }

    return constraints;
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "ORCAConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    return std::make_unique<ORCAConstraint>(*this);
  }

  void setOtherAgents(const std::vector<AgentType *> & other_agents)
  {
    other_agents_ = other_agents;
  }

  void addOtherAgent(AgentType * agent) { other_agents_.push_back(agent); }

  void removeOtherAgent(AgentType * agent)
  {
    other_agents_.erase(
      std::remove(other_agents_.begin(), other_agents_.end(), agent), other_agents_.end());
  }

  auto getTimeHorizon() const noexcept -> double { return time_horizon_; }
  void setTimeHorizon(double time_horizon) { time_horizon_ = time_horizon; }

private:
  std::vector<AgentType *> other_agents_;
  double time_horizon_;
  int priority_;
};

template <Agent AgentType>
class VelocityObstacleConstraint : public ConstraintBase<AgentType>
{
public:
  VelocityObstacleConstraint(
    const AgentType * other_agent, double time_horizon = 1.5, int priority = 20)
  : other_agent_(other_agent), time_horizon_(time_horizon), priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, double /*dt*/) const
    -> std::vector<HalfPlane> override
  {
    std::vector<HalfPlane> constraints;

    if (!other_agent_ || other_agent_->id() == agent.id()) {
      return constraints;
    }

    const auto agent_pos = agent.position();
    const auto other_pos = other_agent_->position();
    const auto other_vel = other_agent_->velocity();
    const auto combined_radius = agent.radius() + other_agent_->radius();

    const auto relative_pos = other_pos - agent_pos;
    const auto distance = relative_pos.norm();

    if (distance < combined_radius + EPSILON) {
      const auto normal = relative_pos.normalized();
      constraints.emplace_back(normal, agent_pos + combined_radius * normal);
      return constraints;
    }

    const auto sin_theta = combined_radius / distance;
    const auto cos_theta = std::sqrt(1.0 - sin_theta * sin_theta);

    const auto unit_relative = relative_pos / distance;
    const auto left_tangent = Vector2{
      unit_relative.x() * cos_theta - unit_relative.y() * sin_theta,
      unit_relative.x() * sin_theta + unit_relative.y() * cos_theta};
    const auto right_tangent = Vector2{
      unit_relative.x() * cos_theta + unit_relative.y() * sin_theta,
      -unit_relative.x() * sin_theta + unit_relative.y() * cos_theta};

    const auto vo_apex = other_vel;
    const auto left_side = vo_apex + left_tangent * 10.0;
    const auto right_side = vo_apex + right_tangent * 10.0;

    constraints.emplace_back(perpendicular(left_tangent), vo_apex);
    constraints.emplace_back(-perpendicular(right_tangent), vo_apex);

    return constraints;
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "VelocityObstacleConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    return std::make_unique<VelocityObstacleConstraint>(*this);
  }

private:
  const AgentType * other_agent_;
  double time_horizon_;
  int priority_;
};

template <Agent AgentType>
class ReciprocalVelocityObstacleConstraint : public ConstraintBase<AgentType>
{
public:
  ReciprocalVelocityObstacleConstraint(
    const AgentType * other_agent, double time_horizon = 2.0, double responsibility_factor = 0.5,
    int priority = 15)
  : other_agent_(other_agent),
    time_horizon_(time_horizon),
    responsibility_factor_(responsibility_factor),
    priority_(priority)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, double dt) const
    -> std::vector<HalfPlane> override
  {
    std::vector<HalfPlane> constraints;

    if (!other_agent_ || other_agent_->id() == agent.id()) {
      return constraints;
    }

    const auto agent_pos = agent.position();
    const auto agent_vel = agent.velocity();
    const auto other_pos = other_agent_->position();
    const auto other_vel = other_agent_->velocity();
    const auto combined_radius = agent.radius() + other_agent_->radius();

    const auto relative_pos = other_pos - agent_pos;
    const auto relative_vel = agent_vel - other_vel;
    const auto distance = relative_pos.norm();

    if (distance < EPSILON) {
      constraints.emplace_back(Vector2{1, 0}, agent_pos + Vector2d{combined_radius, 0});
      return constraints;
    }

    if (distance < combined_radius) {
      const auto normal = relative_pos.normalized();
      const auto penetration_depth = combined_radius - distance;
      const auto separation_vel = normal * (penetration_depth / dt);
      const auto constraint_point = agent_vel + responsibility_factor_ * separation_vel;
      constraints.emplace_back(normal, constraint_point);
      return constraints;
    }

    const auto time_to_collision = distance / relative_vel.norm();
    if (time_to_collision > time_horizon_) {
      return constraints;
    }

    const auto collision_point = agent_pos + agent_vel * time_to_collision;
    const auto other_collision_point = other_pos + other_vel * time_to_collision;
    const auto collision_normal = (collision_point - other_collision_point).normalized();

    const auto avoidance_vel = collision_normal * combined_radius / time_to_collision;
    const auto constraint_point = other_vel + responsibility_factor_ * avoidance_vel;

    constraints.emplace_back(collision_normal, constraint_point);

    return constraints;
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "ReciprocalVelocityObstacleConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    return std::make_unique<ReciprocalVelocityObstacleConstraint>(*this);
  }

  auto getResponsibilityFactor() const noexcept -> double { return responsibility_factor_; }
  void setResponsibilityFactor(double factor)
  {
    responsibility_factor_ = std::clamp(factor, double{0}, double{1});
  }

private:
  const AgentType * other_agent_;
  double time_horizon_;
  double responsibility_factor_;
  int priority_;
};

}  // namespace crane::modern_orca
