#pragma once

#include "../types.hpp"
#include "../concepts.hpp"
#include <memory>

namespace modern_orca {

template<CollisionModel ModelType>
class AgentBase {
public:
    using CollisionModel = ModelType;
    
    AgentBase(AgentId id, const Vector2D& position, const Vector2D& preferred_velocity,
              Scalar max_speed, const ModelType& collision_model)
        : id_(id), position_(position), velocity_(Vector2D{0, 0}), 
          preferred_velocity_(preferred_velocity), max_speed_(max_speed),
          collision_model_(collision_model) {}
    
    virtual ~AgentBase() = default;
    
    auto id() const noexcept -> AgentId { return id_; }
    auto position() const noexcept -> const Vector2D& { return position_; }
    auto velocity() const noexcept -> const Vector2D& { return velocity_; }
    auto preferredVelocity() const noexcept -> const Vector2D& { return preferred_velocity_; }
    auto maxSpeed() const noexcept -> Scalar { return max_speed_; }
    auto radius() const noexcept -> Scalar { return collision_model_.radius(); }
    
    auto collisionModel() const noexcept -> const ModelType& { return collision_model_; }
    auto collisionModel() noexcept -> ModelType& { return collision_model_; }
    
    void setPosition(const Vector2D& position) { position_ = position; }
    void setVelocity(const Vector2D& velocity) { 
        velocity_ = velocity;
        if (velocity_.norm() > max_speed_ + EPSILON) {
            velocity_ = velocity_.normalized() * max_speed_;
        }
    }
    void setPreferredVelocity(const Vector2D& preferred_velocity) { 
        preferred_velocity_ = preferred_velocity; 
    }
    void setMaxSpeed(Scalar max_speed) { max_speed_ = std::max(Scalar{0}, max_speed); }
    
    virtual void update(TimeStep dt) {
        position_ += velocity_ * dt;
    }
    
    auto checkCollisionWith(const AgentBase& other) const -> bool {
        return collision_model_.checkCollision(position_, other.position_);
    }
    
    auto getCollisionConstraintWith(const AgentBase& other) const -> HalfPlaneD {
        return collision_model_.getCollisionConstraint(position_, other.position_);
    }
    
    virtual auto clone() const -> std::unique_ptr<AgentBase> = 0;

protected:
    AgentId id_;
    Vector2D position_;
    Vector2D velocity_;
    Vector2D preferred_velocity_;
    Scalar max_speed_;
    ModelType collision_model_;
};

class CircularCollisionModel {
public:
    explicit CircularCollisionModel(Scalar radius) : radius_(radius) {}
    
    auto radius() const noexcept -> Scalar { return radius_; }
    void setRadius(Scalar radius) { radius_ = std::max(Scalar{0}, radius); }
    
    auto checkCollision(const Vector2D& pos1, const Vector2D& pos2) const -> bool {
        const auto distance = (pos1 - pos2).norm();
        return distance < 2 * radius_ + EPSILON;
    }
    
    auto getCollisionConstraint(const Vector2D& pos1, const Vector2D& pos2) const -> HalfPlaneD {
        const auto relative_pos = pos1 - pos2;
        const auto distance = relative_pos.norm();
        
        if (distance < EPSILON) {
            return HalfPlaneD{Vector2D{1, 0}, pos1 + Vector2D{radius_, 0}};
        }
        
        const auto normal = relative_pos.normalized();
        const auto point = pos2 + normal * radius_;
        return HalfPlaneD{normal, point};
    }

private:
    Scalar radius_;
};

class CircularAgent : public AgentBase<CircularCollisionModel> {
public:
    CircularAgent(AgentId id, const Vector2D& position, const Vector2D& preferred_velocity,
                  Scalar max_speed, Scalar radius)
        : AgentBase(id, position, preferred_velocity, max_speed, 
                   CircularCollisionModel{radius}) {}
    
    auto clone() const -> std::unique_ptr<AgentBase> override {
        return std::make_unique<CircularAgent>(*this);
    }
};

template<std::size_t N>
class PolygonCollisionModel {
public:
    using VertexArray = std::array<Vector2D, N>;
    
    explicit PolygonCollisionModel(const VertexArray& vertices, Scalar radius = 0.0)
        : vertices_(vertices), bounding_radius_(radius) {
        if (bounding_radius_ < EPSILON) {
            calculateBoundingRadius();
        }
    }
    
    auto radius() const noexcept -> Scalar { return bounding_radius_; }
    auto vertices() const noexcept -> const VertexArray& { return vertices_; }
    
    void setVertices(const VertexArray& vertices) {
        vertices_ = vertices;
        calculateBoundingRadius();
    }
    
    auto checkCollision(const Vector2D& pos1, const Vector2D& pos2) const -> bool {
        const auto distance = (pos1 - pos2).norm();
        return distance < 2 * bounding_radius_ + EPSILON;
    }
    
    auto getCollisionConstraint(const Vector2D& pos1, const Vector2D& pos2) const -> HalfPlaneD {
        const auto relative_pos = pos1 - pos2;
        const auto distance = relative_pos.norm();
        
        if (distance < EPSILON) {
            return HalfPlaneD{Vector2D{1, 0}, pos1 + Vector2D{bounding_radius_, 0}};
        }
        
        const auto normal = relative_pos.normalized();
        const auto point = pos2 + normal * bounding_radius_;
        return HalfPlaneD{normal, point};
    }

private:
    VertexArray vertices_;
    Scalar bounding_radius_;
    
    void calculateBoundingRadius() {
        bounding_radius_ = 0;
        for (const auto& vertex : vertices_) {
            bounding_radius_ = std::max(bounding_radius_, vertex.norm());
        }
    }
};

template<std::size_t N>
class PolygonAgent : public AgentBase<PolygonCollisionModel<N>> {
public:
    using VertexArray = typename PolygonCollisionModel<N>::VertexArray;
    
    PolygonAgent(AgentId id, const Vector2D& position, const Vector2D& preferred_velocity,
                 Scalar max_speed, const VertexArray& vertices)
        : AgentBase<PolygonCollisionModel<N>>(id, position, preferred_velocity, max_speed, 
                                             PolygonCollisionModel<N>{vertices}) {}
    
    auto clone() const -> std::unique_ptr<AgentBase<PolygonCollisionModel<N>>> override {
        return std::make_unique<PolygonAgent>(*this);
    }
};

}  // namespace modern_orca