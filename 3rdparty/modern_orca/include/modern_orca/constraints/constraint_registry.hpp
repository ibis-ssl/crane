// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>

#include "../concepts.hpp"
#include "constraint_base.hpp"

namespace crane::modern_orca
{

template <Agent AgentType>
class ConstraintRegistry
{
public:
  using ConstraintFactory = std::function<std::unique_ptr<ConstraintBase<AgentType>>()>;

  static auto instance() -> ConstraintRegistry &
  {
    static ConstraintRegistry registry;
    return registry;
  }

  template <typename ConstraintType, typename... Args>
  static void registerConstraint(const std::string & name)
  {
    static_assert(
      std::is_base_of_v<ConstraintBase<AgentType>, ConstraintType>,
      "ConstraintType must inherit from ConstraintBase");

    auto & registry = instance();
    registry.factories_[name] = []() -> std::unique_ptr<ConstraintBase<AgentType>> {
      return std::make_unique<ConstraintType>();
    };
  }

  template <typename ConstraintType, typename... Args>
  static void registerConstraintWithArgs(const std::string & name, Args &&... args)
  {
    static_assert(
      std::is_base_of_v<ConstraintBase<AgentType>, ConstraintType>,
      "ConstraintType must inherit from ConstraintBase");

    auto & registry = instance();
    registry.factories_[name] = [args...]() -> std::unique_ptr<ConstraintBase<AgentType>> {
      return std::make_unique<ConstraintType>(args...);
    };
  }

  static auto createConstraint(const std::string & name)
    -> std::unique_ptr<ConstraintBase<AgentType>>
  {
    auto & registry = instance();
    auto it = registry.factories_.find(name);
    if (it != registry.factories_.end()) {
      return it->second();
    }
    return nullptr;
  }

  static auto isRegistered(const std::string & name) -> bool
  {
    auto & registry = instance();
    return registry.factories_.find(name) != registry.factories_.end();
  }

  static auto getRegisteredNames() -> std::vector<std::string>
  {
    auto & registry = instance();
    std::vector<std::string> names;
    names.reserve(registry.factories_.size());

    for (const auto & [name, factory] : registry.factories_) {
      names.push_back(name);
    }

    return names;
  }

  static void clear()
  {
    auto & registry = instance();
    registry.factories_.clear();
  }

private:
  ConstraintRegistry() = default;
  std::unordered_map<std::string, ConstraintFactory> factories_;
};

template <Agent AgentType>
class ConstraintManager
{
public:
  using ConstraintPtr = std::unique_ptr<ConstraintBase<AgentType>>;

  template <typename ConstraintType, typename... Args>
  auto addConstraint(Args &&... args) -> ConstraintType *
  {
    static_assert(
      std::is_base_of_v<ConstraintBase<AgentType>, ConstraintType>,
      "ConstraintType must inherit from ConstraintBase");

    auto constraint = std::make_unique<ConstraintType>(std::forward<Args>(args)...);
    auto * ptr = constraint.get();
    constraints_.push_back(std::move(constraint));
    sortConstraints();
    return ptr;
  }

  auto addConstraint(ConstraintPtr constraint) -> ConstraintBase<AgentType> *
  {
    auto * ptr = constraint.get();
    constraints_.push_back(std::move(constraint));
    sortConstraints();
    return ptr;
  }

  auto addConstraintByName(const std::string & name) -> ConstraintBase<AgentType> *
  {
    auto constraint = ConstraintRegistry<AgentType>::createConstraint(name);
    if (constraint) {
      return addConstraint(std::move(constraint));
    }
    return nullptr;
  }

  void removeConstraint(const ConstraintBase<AgentType> * constraint)
  {
    constraints_.erase(
      std::remove_if(
        constraints_.begin(), constraints_.end(),
        [constraint](const auto & ptr) { return ptr.get() == constraint; }),
      constraints_.end());
  }

  void removeConstraintsByName(const std::string & name)
  {
    constraints_.erase(
      std::remove_if(
        constraints_.begin(), constraints_.end(),
        [&name](const auto & ptr) { return ptr->name() == name; }),
      constraints_.end());
  }

  void clear() { constraints_.clear(); }

  auto generateAllHalfPlanes(const AgentType & agent, double dt) const -> std::vector<HalfPlane>
  {
    std::vector<HalfPlane> all_constraints;

    for (const auto & constraint : constraints_) {
      if (constraint->isActive()) {
        auto half_planes = constraint->generateHalfPlanes(agent, dt);
        all_constraints.insert(all_constraints.end(), half_planes.begin(), half_planes.end());
      }
    }

    return all_constraints;
  }

  auto getConstraints() const -> const std::vector<ConstraintPtr> & { return constraints_; }

  auto getConstraintsByName(const std::string & name) const
    -> std::vector<ConstraintBase<AgentType> *>
  {
    std::vector<ConstraintBase<AgentType> *> result;
    for (const auto & constraint : constraints_) {
      if (constraint->name() == name) {
        result.push_back(constraint.get());
      }
    }
    return result;
  }

  template <typename ConstraintType>
  auto getConstraintsOfType() const -> std::vector<ConstraintType *>
  {
    std::vector<ConstraintType *> result;
    for (const auto & constraint : constraints_) {
      if (auto * typed_constraint = dynamic_cast<ConstraintType *>(constraint.get())) {
        result.push_back(typed_constraint);
      }
    }
    return result;
  }

  auto size() const noexcept -> std::size_t { return constraints_.size(); }

  bool empty() const noexcept { return constraints_.empty(); }

private:
  std::vector<ConstraintPtr> constraints_;

  void sortConstraints()
  {
    std::sort(constraints_.begin(), constraints_.end(), [](const auto & a, const auto & b) {
      return a->priority() > b->priority();
    });
  }
};

}  // namespace crane::modern_orca
