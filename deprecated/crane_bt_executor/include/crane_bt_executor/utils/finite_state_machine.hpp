// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__UTILS__FINITE_STATE_MACHINE_HPP_
#define CRANE_BT_EXECUTOR__UTILS__FINITE_STATE_MACHINE_HPP_

#include <functional>
#include <map>
#include <optional>
#include <string>

#include "crane_msg_wrappers/world_model_wrapper.hpp"

struct State
{
  //  State()
  //  std::string name;
  std::function<void(WorldModelWrapper &)> on_enter = nullptr;
  std::function<void(WorldModelWrapper &)> on_update = nullptr;
  std::function<void(WorldModelWrapper &)> on_exit = nullptr;
};

struct StateWithTransition
{
  State state;
  std::map<std::string, std::function<bool(WorldModelWrapper &)>> transitions;
};

class FiniteStateMachine
{
public:
  FiniteStateMachine()
  {
    addState("Start", State());
    addState("Success", State());
    addState("Failed", State());
  }

  void addState(std::string state_name, State state)
  {
    StateWithTransition st;
    st.state = state;
    states_[state_name] = st;
  }

  bool addTransition(
    std::string from_state_name, std::string to_state_name,
    std::function<bool(WorldModelWrapper &)> judge_func)
  {
    decltype(states_.find(from_state_name)) from_state_it;
    try {
      from_state_it = states_.find(from_state_name);
    } catch (std::out_of_range & oor) {
      return false;
    }

    from_state_it->second.transitions[to_state_name] = judge_func;

    return true;
  }
  void update(WorldModelWrapper & world_model)
  {
    // TODO(HansRobo) : Imprement
    decltype(states_.find(current_state_)) state;
    try {
      state = states_.find(current_state_);
    } catch (std::out_of_range & oor) {
      return;
    }

    // state update
    if (state->second.state.on_update) {
      state->second.state.on_update(world_model);
    }

    // check for transition
    for (auto transition : state->second.transitions) {
      bool result = transition.second(world_model);
      if (result) {
        auto name = transition.first;
        // find next state
        decltype(states_.find(name)) next_state;
        try {
          next_state = states_.find(name);
        } catch (std::out_of_range & oor) {
          return;
        }

        // transition
        // finalize
        if (state->second.state.on_exit) {
          state->second.state.on_exit(world_model);
        }
        // initialize
        if (next_state->second.state.on_enter) {
          next_state->second.state.on_enter(world_model);
        }
        current_state_ = name;
      }
    }
  }

  void executeState(State & state) {}

  bool transitionTo(std::string name)
  {
    decltype(states_.find(name)) next_state;
    try {
      next_state = states_.find(name);
    } catch (std::out_of_range & oor) {
      return false;
    }

    return true;
  }

private:
  std::map<std::string, StateWithTransition> states_;
  std::string current_state_ = "Start";
};
#endif  // CRANE_BT_EXECUTOR__UTILS__FINITE_STATE_MACHINE_HPP_
