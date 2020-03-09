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

struct State
{
//  State()
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
  template<typename TEnum>
  void addState(TEnum id, State state)
  {
    StateWithTransition st;
    st.state = state;
    states[static_cast<uint8_t>(id)] = st;
  }

  template<typename TEnum>
  bool addTransition(TEnum from_id, TEnum to_id, std::function<bool(WorldModel &)> judge_func)
  {
    auto from_state_id = static_cast<uint8_t>(from_id);
    auto to_state_id = static_cast<uint8_t>(to_id);

    decltype(states.find(from_state_id)) from_state_it;
    try {
      from_state_it = states.find(from_state_id);
    } catch (std::out_of_range & oor) {return false;}

    from_state_it->second.transitions[to_state_id] = judge_func;

    return true;
  }

private:
  std::map<uint8_t, StateWithTransition> states;
};
#endif  // CRANE_BT_EXECUTOR__UTILS__FINITE_STATE_MACHINE_HPP_
