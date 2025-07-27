// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <concepts>
#include <string>
#include <vector>

#include "types.hpp"

namespace crane::modern_orca
{

template <typename T>
concept Agent = requires(T agent, Vector2 pos, Vector2 vel, double dt) {
  typename T::CollisionModel;

  { agent.position() } -> std::convertible_to<Vector2>;
  { agent.velocity() } -> std::convertible_to<Vector2>;
  { agent.preferredVelocity() } -> std::convertible_to<Vector2>;
  { agent.maxSpeed() } -> std::convertible_to<double>;
  { agent.radius() } -> std::convertible_to<double>;

  { agent.setPosition(pos) } -> std::same_as<void>;
  { agent.setVelocity(vel) } -> std::same_as<void>;
  { agent.setPreferredVelocity(vel) } -> std::same_as<void>;

  { agent.update(dt) } -> std::same_as<void>;
  { agent.id() } -> std::convertible_to<AgentId>;
};

template <typename T, typename AgentType>
concept Constraint = requires(T constraint, const AgentType & agent, double dt) {
  { constraint.generateHalfPlanes(agent, dt) } -> std::convertible_to<std::vector<HalfPlane>>;
  { constraint.priority() } -> std::convertible_to<int>;
  { constraint.isActive() } -> std::convertible_to<bool>;
};

template <typename T>
concept Solver = requires(T solver, const std::vector<HalfPlane> & constraints, Vector2 preferred) {
  { solver.solve(constraints, preferred) } -> std::convertible_to<Vector2>;
  { solver.feasible() } -> std::convertible_to<bool>;
  { solver.iterations() } -> std::convertible_to<std::size_t>;
};

template <typename T>
concept CollisionModel = requires(T model, Vector2 pos1, Vector2 pos2) {
  { model.checkCollision(pos1, pos2) } -> std::convertible_to<bool>;
  { model.getCollisionConstraint(pos1, pos2) } -> std::convertible_to<HalfPlane>;
  { model.radius() } -> std::convertible_to<double>;
};

template <typename T>
concept ConstraintProvider = requires(T provider) {
  typename T::ConstraintType;
  { provider.createConstraint() } -> std::convertible_to<typename T::ConstraintType>;
  { provider.name() } -> std::convertible_to<std::string>;
};

template <typename T>
concept Serializable = requires(T obj) {
  { obj.serialize() } -> std::convertible_to<std::string>;
  { T::deserialize(std::string{}) } -> std::convertible_to<T>;
};

template <typename T>
concept Configurable = requires(T config) {
  { config.validate() } -> std::convertible_to<bool>;
  { config.getParameter(std::string{}) } -> std::convertible_to<double>;
  { config.setParameter(std::string{}, double{}) } -> std::same_as<void>;
};

}  // namespace crane::modern_orca
