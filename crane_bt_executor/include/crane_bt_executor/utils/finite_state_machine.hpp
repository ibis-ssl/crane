// Copyright (c) 2020 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef CRANE_BT_EXECUTOR__UTILS__FINITE_STATE_MACHINE_HPP_
#define CRANE_BT_EXECUTOR__UTILS__FINITE_STATE_MACHINE_HPP_

#include <crane_world_observer/world_model.hpp>
#include <optional>
#include <functional>
#include <map>
#include <string>


struct State
{
//  State()
//  std::string name;
  std::function<void(WorldModel &)> on_enter = nullptr;
  std::function<void(WorldModel &)> on_update = nullptr;
  std::function<void(WorldModel &)> on_exit = nullptr;
};


struct StateWithTransition
{
  State state;
  std::map<std::string, std::function<bool(WorldModel &)>> transitions;
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
    std::function<bool(WorldModel &)> judge_func)
  {
    decltype(states_.find(from_state_name)) from_state_it;
    try {
      from_state_it = states_.find(from_state_name);
    } catch (std::out_of_range & oor) {return false;}

    from_state_it->second.transitions[to_state_name] = judge_func;

    return true;
  }
  void update(WorldModel & world_model)
  {
    // TODO(HansRobo) : Imprement
    decltype(states_.find(current_state_)) state;
    try {
      state = states_.find(current_state_);
    } catch (std::out_of_range & oor) {return;}

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
        } catch (std::out_of_range & oor) {return;}

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

  void executeState(State & state)
  {
  }

  bool transitionTo(std::string name)
  {
    decltype(states_.find(name)) next_state;
    try {
      next_state = states_.find(name);
    } catch (std::out_of_range & oor) {return false;}

    return true;
  }

private:
  std::map<std::string, StateWithTransition> states_;
  std::string current_state_ = "Start";
};
#endif  // CRANE_BT_EXECUTOR__UTILS__FINITE_STATE_MACHINE_HPP_
