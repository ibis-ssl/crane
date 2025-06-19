// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <memory>

#include "../concepts.hpp"
#include "../types.hpp"

namespace crane::modern_orca
{

template <CollisionModel ModelType>
class AgentBase
{
public:
  using CollisionModel = ModelType;

  AgentBase(
    AgentId id, const Vector2 & position, const Vector2 & preferred_velocity, double max_speed,
    const ModelType & collision_model)
  : id_(id),
    position_(position),
    velocity_(Vector2{0, 0}),
    preferred_velocity_(preferred_velocity),
    max_speed_(max_speed),
    collision_model_(collision_model)
  {
  }

  virtual ~AgentBase() = default;

  auto id() const noexcept -> AgentId { return id_; }
  auto position() const noexcept -> const Vector2 & { return position_; }
  auto velocity() const noexcept -> const Vector2 & { return velocity_; }
  auto preferredVelocity() const noexcept -> const Vector2 & { return preferred_velocity_; }
  auto maxSpeed() const noexcept -> double { return max_speed_; }
  auto radius() const noexcept -> double { return collision_model_.radius(); }

  auto collisionModel() const noexcept -> const ModelType & { return collision_model_; }
  auto collisionModel() noexcept -> ModelType & { return collision_model_; }

  void setPosition(const Vector2 & position) { position_ = position; }
  void setVelocity(const Vector2 & velocity)
  {
    velocity_ = velocity;
    if (velocity_.norm() > max_speed_ + EPSILON) {
      velocity_ = velocity_.normalized() * max_speed_;
    }
  }
  void setPreferredVelocity(const Vector2 & preferred_velocity)
  {
    preferred_velocity_ = preferred_velocity;
  }
  void setMaxSpeed(double max_speed) { max_speed_ = std::max(double{0}, max_speed); }

  virtual void update(double dt) { position_ += velocity_ * dt; }

  auto checkCollisionWith(const AgentBase & other) const -> bool
  {
    return collision_model_.checkCollision(position_, other.position_);
  }

  auto getCollisionConstraintWith(const AgentBase & other) const -> HalfPlane
  {
    return collision_model_.getCollisionConstraint(position_, other.position_);
  }

  virtual auto clone() const -> std::unique_ptr<AgentBase> = 0;

protected:
  AgentId id_;
  Vector2 position_;
  Vector2 velocity_;
  Vector2 preferred_velocity_;
  double max_speed_;
  ModelType collision_model_;
};

class CircularCollisionModel
{
public:
  explicit CircularCollisionModel(double radius) : radius_(radius) {}

  auto radius() const noexcept -> double { return radius_; }
  void setRadius(double radius) { radius_ = std::max(double{0}, radius); }

  auto checkCollision(const Vector2 & pos1, const Vector2 & pos2) const -> bool
  {
    const auto distance = (pos1 - pos2).norm();
    return distance < 2 * radius_ + EPSILON;
  }

  auto getCollisionConstraint(const Vector2 & pos1, const Vector2 & pos2) const -> HalfPlane
  {
    const auto relative_pos = pos1 - pos2;
    const auto distance = relative_pos.norm();

    if (distance < EPSILON) {
      return HalfPlane{Vector2{1, 0}, pos1 + Vector2{radius_, 0}};
    }

    const auto normal = relative_pos.normalized();
    const auto point = pos2 + normal * radius_;
    return HalfPlane{normal, point};
  }

private:
  double radius_;
};

class CircularAgent : public AgentBase<CircularCollisionModel>
{
public:
  CircularAgent(
    AgentId id, const Vector2 & position, const Vector2 & preferred_velocity, double max_speed,
    double radius)
  : AgentBase(id, position, preferred_velocity, max_speed, CircularCollisionModel{radius})
  {
  }

  auto clone() const -> std::unique_ptr<AgentBase> override
  {
    return std::make_unique<CircularAgent>(*this);
  }
};

template <std::size_t N>
class PolygonCollisionModel
{
public:
  using VertexArray = std::array<Vector2, N>;

  explicit PolygonCollisionModel(const VertexArray & vertices, double radius = 0.0)
  : vertices_(vertices), bounding_radius_(radius)
  {
    if (bounding_radius_ < EPSILON) {
      calculateBoundingRadius();
    }
  }

  auto radius() const noexcept -> double { return bounding_radius_; }
  auto vertices() const noexcept -> const VertexArray & { return vertices_; }

  void setVertices(const VertexArray & vertices)
  {
    vertices_ = vertices;
    calculateBoundingRadius();
  }

  auto checkCollision(const Vector2 & pos1, const Vector2 & pos2) const -> bool
  {
    const auto distance = (pos1 - pos2).norm();
    return distance < 2 * bounding_radius_ + EPSILON;
  }

  auto getCollisionConstraint(const Vector2 & pos1, const Vector2 & pos2) const -> HalfPlane
  {
    const auto relative_pos = pos1 - pos2;
    const auto distance = relative_pos.norm();

    if (distance < EPSILON) {
      return HalfPlane{Vector2{1, 0}, pos1 + Vector2{bounding_radius_, 0}};
    }

    const auto normal = relative_pos.normalized();
    const auto point = pos2 + normal * bounding_radius_;
    return HalfPlane{normal, point};
  }

private:
  VertexArray vertices_;
  double bounding_radius_;

  void calculateBoundingRadius()
  {
    bounding_radius_ = 0;
    for (const auto & vertex : vertices_) {
      bounding_radius_ = std::max(bounding_radius_, vertex.norm());
    }
  }
};

template <std::size_t N>
class PolygonAgent : public AgentBase<PolygonCollisionModel<N>>
{
public:
  using VertexArray = typename PolygonCollisionModel<N>::VertexArray;

  PolygonAgent(
    AgentId id, const Vector2 & position, const Vector2 & preferred_velocity, double max_speed,
    const VertexArray & vertices)
  : AgentBase<PolygonCollisionModel<N>>(
      id, position, preferred_velocity, max_speed, PolygonCollisionModel<N>{vertices})
  {
  }

  auto clone() const -> std::unique_ptr<AgentBase<PolygonCollisionModel<N>>> override
  {
    return std::make_unique<PolygonAgent>(*this);
  }
};

}  // namespace crane::modern_orca
