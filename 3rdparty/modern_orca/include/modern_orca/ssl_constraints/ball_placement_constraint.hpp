// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "ssl_constraint_base.hpp"
#include <crane_basics/geometry_operations.hpp>

namespace crane::modern_orca
{

template <Agent AgentType>
class BallPlacementAvoidanceConstraint : public SSLConstraintBase<AgentType>
{
public:
  explicit BallPlacementAvoidanceConstraint(int priority = 85)
  : priority_(priority), placement_area_offset_(0.8)
  {
  }

  auto generateHalfPlanes(const AgentType & agent, double /*dt*/) const
    -> std::vector<HalfPlane> override
  {
    std::vector<HalfPlane> constraints;

    if (!world_model_ || !this->isEnabled()) {
      return constraints;
    }

    // Check if ball placement is active
    if (!world_model_->getBallPlacementTarget().has_value()) {
      return constraints;
    }

    const auto agent_pos = agent.position();
    const auto agent_radius = agent.radius();
    
    auto placement_area = world_model_->getBallPlacementArea();
    if (!placement_area.has_value()) {
      return constraints;
    }

    // Check if agent is within placement area
    const auto distance_to_area = bg::distance(
      Point(agent_pos.x(), agent_pos.y()), 
      placement_area.value()
    );
    
    const auto required_distance = placement_area.value().radius + placement_area_offset_ + agent_radius;
    
    if (distance_to_area < required_distance) {
      // Find closest point on placement area segment
      const auto segment = placement_area.value().segment;
      Point segment_start(segment.first.x(), segment.first.y());
      Point segment_end(segment.second.x(), segment.second.y());
      
      auto [distance, closest_point] = crane::getClosestPointAndDistance(
        Point(agent_pos.x(), agent_pos.y()),
        crane::Segment(segment_start, segment_end)
      );
      
      // Calculate direction away from placement area
      Vector2 direction = agent_pos - Vector2(closest_point.x(), closest_point.y());
      if (direction.norm() < EPSILON) {
        // If agent is on the line, use perpendicular direction
        Vector2 line_dir = Vector2(segment_end.x() - segment_start.x(), segment_end.y() - segment_start.y()).normalized();
        direction = Vector2(-line_dir.y(), line_dir.x()); // Perpendicular
      } else {
        direction = direction.normalized();
      }
      
      // Create constraint point at required distance
      Vector2 constraint_point = Vector2(closest_point.x(), closest_point.y()) + direction * required_distance;
      constraints.emplace_back(direction, constraint_point);
    }

    return constraints;
  }

  void updateFromWorldModel(const crane::WorldModelWrapper::SharedPtr & world_model) override
  {
    world_model_ = world_model;
  }

  void updateFromRefereeCommand(const robocup_ssl_msgs::msg::Referee::_command_type & command) override
  {
    // Ball placement constraint is active during ball placement commands
    // The constraint itself checks if placement target exists
  }

  auto priority() const noexcept -> int override { return priority_; }
  auto name() const -> std::string override { return "BallPlacementAvoidanceConstraint"; }

  auto clone() const -> std::unique_ptr<ConstraintBase<AgentType>> override
  {
    auto cloned = std::make_unique<BallPlacementAvoidanceConstraint>(*this);
    cloned->world_model_ = world_model_;
    return cloned;
  }

  SSLConstraintType getConstraintType() const noexcept override
  {
    return SSLConstraintType::BALL_PLACEMENT_AVOIDANCE;
  }

  // Ball placement specific configuration
  void setPlacementAreaOffset(double offset) { placement_area_offset_ = offset; }
  double getPlacementAreaOffset() const { return placement_area_offset_; }

protected:
  bool isConstraintActive() const noexcept override
  {
    return world_model_ != nullptr && world_model_->getBallPlacementTarget().has_value();
  }

private:
  crane::WorldModelWrapper::SharedPtr world_model_;
  int priority_;
  double placement_area_offset_;
};

}  // namespace crane::modern_orca
