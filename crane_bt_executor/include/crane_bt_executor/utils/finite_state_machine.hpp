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

#include <crane_bt_executor/utils/world_model.hpp>
#include <functional>
#include <map>
#include <string>
#include <optional>

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
    states[state_name] = st;
  }

  bool addTransition(
    std::string from_state_name, std::string to_state_name,
    std::function<bool(WorldModel &)> judge_func)
  {
    decltype(states.find(from_state_name)) from_state_it;
    try {
      from_state_it = states.find(from_state_name);
    } catch (std::out_of_range & oor) {return false;}

    from_state_it->second.transitions[to_state_name] = judge_func;

    return true;
  }
  void update(WorldModel & world_model)
  {
    // TODO(HansRobo) : Imprement
    decltype(states.find(current_state)) state;
    try {
      state = states.find(current_state);
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
        decltype(states.find(name)) next_state;
        try {
          next_state = states.find(name);
        } catch (std::out_of_range & oor) {return;}

        // transition
        // finalize
        if(state->second.state.on_exit){
          state->second.state.on_exit(world_model);
        }
        // initialize
        if(next_state->second.state.on_enter){
          next_state->second.state.on_enter(world_model);
        }
        current_state = name;
      }
    }
  }

  void executeState(State & state)
  {
  }

  bool transitionTo(std::string name){
    decltype(states.find(name)) next_state;
    try {
      next_state = states.find(name);
    } catch (std::out_of_range & oor) {return false;}

    return true;
  }

private:
  std::map<std::string, StateWithTransition> states;
  std::string current_state = "Start";
};
#endif  // CRANE_BT_EXECUTOR__UTILS__FINITE_STATE_MACHINE_HPP_
