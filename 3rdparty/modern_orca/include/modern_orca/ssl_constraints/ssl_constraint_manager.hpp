// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "../constraints/constraint_registry.hpp"
#include "ball_avoidance_constraint.hpp"
#include "ball_placement_constraint.hpp"
#include "penalty_area_constraint.hpp"
#include "referee_command_constraint.hpp"
#include "ssl_constraint_base.hpp"

namespace crane::modern_orca
{

template <Agent AgentType>
class SSLConstraintManager
{
public:
  using ConstraintPtr = std::unique_ptr<SSLConstraintBase<AgentType>>;

  struct ConstraintLineup
  {
    bool ball_avoidance_enabled = true;
    bool penalty_area_avoidance_enabled = true;
    bool ball_placement_avoidance_enabled = true;
    bool referee_command_enabled = true;
    bool robot_collision_enabled = true;
  };

  SSLConstraintManager()
  {
    initializeConstraints();
  }

  // Update all constraints from world model
  void updateFromWorldModel(const crane::WorldModelWrapper::SharedPtr & world_model)
  {
    world_model_ = world_model;
    for (auto & [type, constraint] : constraints_) {
      constraint->updateFromWorldModel(world_model);
    }
  }

  // Update all constraints from referee command
  void updateFromRefereeCommand(const robocup_ssl_msgs::msg::Referee::_command_type & command)
  {
    current_referee_command_ = command;
    for (auto & [type, constraint] : constraints_) {
      constraint->updateFromRefereeCommand(command);
    }
  }

  // Configure constraint lineup (which constraints are enabled)
  void setConstraintLineup(const ConstraintLineup & lineup)
  {
    lineup_ = lineup;
    updateConstraintStates();
  }

  const ConstraintLineup & getConstraintLineup() const { return lineup_; }

  // Enable/disable individual constraint types
  void setConstraintEnabled(SSLConstraintType type, bool enabled)
  {
    switch (type) {
      case SSLConstraintType::BALL_AVOIDANCE:
        lineup_.ball_avoidance_enabled = enabled;
        break;
      case SSLConstraintType::PENALTY_AREA_AVOIDANCE:
        lineup_.penalty_area_avoidance_enabled = enabled;
        break;
      case SSLConstraintType::BALL_PLACEMENT_AVOIDANCE:
        lineup_.ball_placement_avoidance_enabled = enabled;
        break;
      case SSLConstraintType::REFEREE_COMMAND:
        lineup_.referee_command_enabled = enabled;
        break;
      case SSLConstraintType::ROBOT_COLLISION:
        lineup_.robot_collision_enabled = enabled;
        break;
    }
    updateConstraintStates();
  }

  bool isConstraintEnabled(SSLConstraintType type) const
  {
    switch (type) {
      case SSLConstraintType::BALL_AVOIDANCE:
        return lineup_.ball_avoidance_enabled;
      case SSLConstraintType::PENALTY_AREA_AVOIDANCE:
        return lineup_.penalty_area_avoidance_enabled;
      case SSLConstraintType::BALL_PLACEMENT_AVOIDANCE:
        return lineup_.ball_placement_avoidance_enabled;
      case SSLConstraintType::REFEREE_COMMAND:
        return lineup_.referee_command_enabled;
      case SSLConstraintType::ROBOT_COLLISION:
        return lineup_.robot_collision_enabled;
      default:
        return false;
    }
  }

  // Generate all active constraints for an agent
  auto generateAllHalfPlanes(const AgentType & agent, double dt) const -> std::vector<HalfPlane>
  {
    std::vector<HalfPlane> all_constraints;

    for (const auto & [type, constraint] : constraints_) {
      if (constraint->isActive()) {
        auto half_planes = constraint->generateHalfPlanes(agent, dt);
        all_constraints.insert(all_constraints.end(), half_planes.begin(), half_planes.end());
      }
    }

    return all_constraints;
  }

  // Get specific constraint by type
  template <typename ConstraintClass>
  ConstraintClass * getConstraint(SSLConstraintType type)
  {
    auto it = constraints_.find(type);
    if (it != constraints_.end()) {
      return dynamic_cast<ConstraintClass *>(it->second.get());
    }
    return nullptr;
  }

  // Get all constraints
  const std::unordered_map<SSLConstraintType, ConstraintPtr> & getConstraints() const
  {
    return constraints_;
  }

  // Apply constraint lineup to disable/enable constraints based on referee command
  void applyAutomaticConstraintAdjustments()
  {
    if (!world_model_) return;

    // Automatic adjustments based on game situation
    switch (current_referee_command_) {
      case robocup_ssl_msgs::msg::Referee::COMMAND_HALT:
        // During HALT, only referee command constraint is active
        setConstraintEnabled(SSLConstraintType::BALL_AVOIDANCE, false);
        setConstraintEnabled(SSLConstraintType::PENALTY_AREA_AVOIDANCE, false);
        setConstraintEnabled(SSLConstraintType::BALL_PLACEMENT_AVOIDANCE, false);
        setConstraintEnabled(SSLConstraintType::REFEREE_COMMAND, true);
        break;

      case robocup_ssl_msgs::msg::Referee::COMMAND_STOP:
        // During STOP, enable all constraints with stricter parameters
        setConstraintEnabled(SSLConstraintType::BALL_AVOIDANCE, true);
        setConstraintEnabled(SSLConstraintType::PENALTY_AREA_AVOIDANCE, true);
        setConstraintEnabled(SSLConstraintType::BALL_PLACEMENT_AVOIDANCE, true);
        setConstraintEnabled(SSLConstraintType::REFEREE_COMMAND, true);
        break;

      default:
        // Normal play: enable all constraints
        setConstraintEnabled(SSLConstraintType::BALL_AVOIDANCE, true);
        setConstraintEnabled(SSLConstraintType::PENALTY_AREA_AVOIDANCE, true);
        setConstraintEnabled(SSLConstraintType::BALL_PLACEMENT_AVOIDANCE, true);
        setConstraintEnabled(SSLConstraintType::REFEREE_COMMAND, true);
        break;
    }
  }

private:
  void initializeConstraints()
  {
    constraints_[SSLConstraintType::BALL_AVOIDANCE] =
      std::make_unique<BallAvoidanceConstraint<AgentType>>();
    constraints_[SSLConstraintType::PENALTY_AREA_AVOIDANCE] =
      std::make_unique<PenaltyAreaAvoidanceConstraint<AgentType>>();
    constraints_[SSLConstraintType::BALL_PLACEMENT_AVOIDANCE] =
      std::make_unique<BallPlacementAvoidanceConstraint<AgentType>>();
    constraints_[SSLConstraintType::REFEREE_COMMAND] =
      std::make_unique<RefereeCommandConstraint<AgentType>>();
  }

  void updateConstraintStates()
  {
    for (auto & [type, constraint] : constraints_) {
      constraint->setEnabled(isConstraintEnabled(type));
    }
  }

  std::unordered_map<SSLConstraintType, ConstraintPtr> constraints_;
  ConstraintLineup lineup_;
  crane::WorldModelWrapper::SharedPtr world_model_;
  robocup_ssl_msgs::msg::Referee::_command_type current_referee_command_ = 
    robocup_ssl_msgs::msg::Referee::COMMAND_NORMAL_START;
};

// Convenience alias for CircularAgent
using SSLConstraintManagerForCircularAgent = SSLConstraintManager<class CircularAgent>;

}  // namespace crane::modern_orca
