// Copyright 2024 ibis-ssl
#pragma once

#include <concepts>
#include <string>
#include <vector>

#include "types.hpp"

namespace modern_orca
{

template <typename T>
concept Agent = requires(T agent, Vector2D pos, Vector2D vel, TimeStep dt) {
  typename T::CollisionModel;

  { agent.position() } -> std::convertible_to<Vector2D>;
  { agent.velocity() } -> std::convertible_to<Vector2D>;
  { agent.preferredVelocity() } -> std::convertible_to<Vector2D>;
  { agent.maxSpeed() } -> std::convertible_to<Scalar>;
  { agent.radius() } -> std::convertible_to<Scalar>;

  { agent.setPosition(pos) } -> std::same_as<void>;
  { agent.setVelocity(vel) } -> std::same_as<void>;
  { agent.setPreferredVelocity(vel) } -> std::same_as<void>;

  { agent.update(dt) } -> std::same_as<void>;
  { agent.id() } -> std::convertible_to<AgentId>;
};

template <typename T, typename AgentType>
concept Constraint = requires(T constraint, const AgentType & agent, TimeStep dt) {
  { constraint.generateHalfPlanes(agent, dt) } -> std::convertible_to<std::vector<HalfPlaneD>>;
  { constraint.priority() } -> std::convertible_to<int>;
  { constraint.isActive() } -> std::convertible_to<bool>;
};

template <typename T>
concept Solver =
  requires(T solver, const std::vector<HalfPlaneD> & constraints, Vector2D preferred) {
    { solver.solve(constraints, preferred) } -> std::convertible_to<Vector2D>;
    { solver.feasible() } -> std::convertible_to<bool>;
    { solver.iterations() } -> std::convertible_to<std::size_t>;
  };

template <typename T>
concept CollisionModel = requires(T model, Vector2D pos1, Vector2D pos2) {
  { model.checkCollision(pos1, pos2) } -> std::convertible_to<bool>;
  { model.getCollisionConstraint(pos1, pos2) } -> std::convertible_to<HalfPlaneD>;
  { model.radius() } -> std::convertible_to<Scalar>;
};

template <typename T>
concept ConstraintProvider = requires(T provider) {
  typename T::ConstraintType;
  { provider.createConstraint() } -> std::convertible_to<typename T::ConstraintType>;
  { provider.name() } -> std::convertible_to<std::string>;
};

template <typename T>
concept Visualizable = requires(T obj) {
  { obj.getVisualizationData() }
};

template <typename T>
concept Serializable = requires(T obj) {
  { obj.serialize() } -> std::convertible_to<std::string>;
  { T::deserialize(std::string{}) } -> std::convertible_to<T>;
};

template <typename T>
concept Configurable = requires(T config) {
  { config.validate() } -> std::convertible_to<bool>;
  { config.getParameter(std::string{}) } -> std::convertible_to<Scalar>;
  { config.setParameter(std::string{}, Scalar{}) } -> std::same_as<void>;
};

}  // namespace modern_orca
