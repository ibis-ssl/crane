// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <algorithm>
#include <execution>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "agents/agent_base.hpp"
#include "concepts.hpp"
#include "constraints/constraint_registry.hpp"
#include "constraints/orca_constraint.hpp"
#include "solvers/solver_base.hpp"
#include "types.hpp"

namespace modern_orca
{

template <Agent AgentType>
class Simulator
{
public:
  using agent_type = AgentType;
  using constraint_manager_type = ConstraintManager<AgentType>;
  using solver_type = LinearProgram2DSolver;

  explicit Simulator(
    std::unique_ptr<SolverBase<solver_type>> solver = std::make_unique<LinearProgram2DSolver>())
  : solver_(std::move(solver)), time_(0.0), parallel_execution_(true)
  {
  }

  template <typename... Args>
  auto addAgent(Args &&... args) -> AgentId
  {
    auto agent = std::make_unique<AgentType>(next_agent_id_, std::forward<Args>(args)...);
    auto id = agent->id();
    agents_[id] = std::move(agent);
    agent_constraints_[id] = std::make_unique<constraint_manager_type>();
    next_agent_id_++;
    return id;
  }

  void removeAgent(AgentId id)
  {
    agents_.erase(id);
    agent_constraints_.erase(id);

    for (auto & [_, constraints] : agent_constraints_) {
      auto orca_constraints =
        constraints->template getConstraintsOfType<ORCAConstraint<AgentType>>();
      for (auto * orca : orca_constraints) {
        orca->removeOtherAgent(getAgent(id));
      }
    }
  }

  auto getAgent(AgentId id) -> AgentType &
  {
    auto it = agents_.find(id);
    if (it == agents_.end()) {
      throw std::out_of_range("Agent not found");
    }
    return *it->second;
  }

  auto getAgent(AgentId id) const -> const AgentType &
  {
    auto it = agents_.find(id);
    if (it == agents_.end()) {
      throw std::out_of_range("Agent not found");
    }
    return *it->second;
  }

  template <typename ConstraintType, typename... Args>
  auto addConstraint(AgentId agent_id, Args &&... args) -> ConstraintType *
  {
    auto it = agent_constraints_.find(agent_id);
    if (it == agent_constraints_.end()) {
      throw std::out_of_range("Agent not found");
    }
    return it->second->template addConstraint<ConstraintType>(std::forward<Args>(args)...);
  }

  template <typename ConstraintType, typename... Args>
  auto addGlobalConstraint(Args &&... args) -> std::vector<ConstraintType *>
  {
    std::vector<ConstraintType *> added_constraints;
    added_constraints.reserve(agents_.size());

    for (auto & [id, _] : agents_) {
      added_constraints.push_back(addConstraint<ConstraintType>(id, std::forward<Args>(args)...));
    }

    return added_constraints;
  }

  auto addORCAConstraints() -> void
  {
    std::vector<AgentType *> agent_pointers;
    agent_pointers.reserve(agents_.size());

    for (auto & [id, agent] : agents_) {
      agent_pointers.push_back(agent.get());
    }

    for (auto & [id, constraints] : agent_constraints_) {
      constraints->template addConstraint<ORCAConstraint<AgentType>>(agent_pointers);
    }
  }

  void step(TimeStep dt)
  {
    if (parallel_execution_ && agents_.size() > 4) {
      stepParallel(dt);
    } else {
      stepSequential(dt);
    }

    time_ += dt;
  }

  auto getAllAgents() const -> const std::unordered_map<AgentId, std::unique_ptr<AgentType>> &
  {
    return agents_;
  }

  auto getConstraintManager(AgentId id) -> constraint_manager_type &
  {
    auto it = agent_constraints_.find(id);
    if (it == agent_constraints_.end()) {
      throw std::out_of_range("Agent not found");
    }
    return *it->second;
  }

  auto getConstraintManager(AgentId id) const -> const constraint_manager_type &
  {
    auto it = agent_constraints_.find(id);
    if (it == agent_constraints_.end()) {
      throw std::out_of_range("Agent not found");
    }
    return *it->second;
  }

  auto currentTime() const noexcept -> Scalar { return time_; }
  void setTime(Scalar time) { time_ = time; }

  auto agentCount() const noexcept -> std::size_t { return agents_.size(); }
  bool empty() const noexcept { return agents_.empty(); }

  void setSolver(std::unique_ptr<SolverBase<solver_type>> solver) { solver_ = std::move(solver); }

  auto getSolver() const -> const SolverBase<solver_type> & { return *solver_; }

  void setParallelExecution(bool enable) { parallel_execution_ = enable; }
  auto isParallelExecution() const noexcept -> bool { return parallel_execution_; }

  void clear()
  {
    agents_.clear();
    agent_constraints_.clear();
    time_ = 0.0;
    next_agent_id_ = 0;
  }

  struct SimulationStatistics
  {
    std::size_t agent_count = 0;
    std::size_t total_constraints = 0;
    Scalar current_time = 0.0;
  };

  auto getStatistics() const -> SimulationStatistics
  {
    SimulationStatistics stats;
    stats.agent_count = agents_.size();
    stats.total_constraints = 0;
    stats.current_time = time_;

    for (const auto & [id, constraints] : agent_constraints_) {
      stats.total_constraints += constraints->size();
    }

    return stats;
  }

private:
  std::unordered_map<AgentId, std::unique_ptr<AgentType>> agents_;
  std::unordered_map<AgentId, std::unique_ptr<constraint_manager_type>> agent_constraints_;
  std::unique_ptr<SolverBase<solver_type>> solver_;
  AgentId next_agent_id_ = 0;
  Scalar time_;
  bool parallel_execution_;

  void stepSequential(TimeStep dt)
  {
    for (auto & [id, agent] : agents_) {
      auto & constraints = *agent_constraints_[id];
      auto half_planes = constraints.generateAllHalfPlanes(*agent, dt);

      solver_->reset();
      auto optimal_velocity = solver_->solve(half_planes, agent->preferredVelocity());

      agent->setVelocity(optimal_velocity);
      agent->update(dt);
    }
  }

  void stepParallel(TimeStep dt)
  {
    std::vector<std::pair<AgentId, Vector2D>> velocity_updates;
    velocity_updates.resize(agents_.size());

    std::vector<AgentId> agent_ids;
    agent_ids.reserve(agents_.size());
    for (const auto & [id, _] : agents_) {
      agent_ids.push_back(id);
    }

    std::transform(
      std::execution::par_unseq, agent_ids.begin(), agent_ids.end(), velocity_updates.begin(),
      [this, dt](AgentId id) -> std::pair<AgentId, Vector2D> {
        auto & agent = *agents_.at(id);
        auto & constraints = *agent_constraints_.at(id);
        auto half_planes = constraints.generateAllHalfPlanes(agent, dt);

        auto local_solver = solver_->clone();
        local_solver->reset();
        auto optimal_velocity = local_solver->solve(half_planes, agent.preferredVelocity());

        return {id, optimal_velocity};
      });

    for (const auto & [id, velocity] : velocity_updates) {
      agents_[id]->setVelocity(velocity);
      agents_[id]->update(dt);
    }
  }
};

using CircularAgentSimulator = Simulator<CircularAgent>;

template <std::size_t N>
using PolygonAgentSimulator = Simulator<PolygonAgent<N>>;

}  // namespace modern_orca
