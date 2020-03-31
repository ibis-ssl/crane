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
  std::map<uint8_t, std::function<bool(WorldModel &)>> transitions;
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
  void update()
  {
    // TODO(HansRobo) : Imprement

  }

private:
  std::map<std::string, StateWithTransition> states;
};
#endif  // CRANE_BT_EXECUTOR__UTILS__FINITE_STATE_MACHINE_HPP_
