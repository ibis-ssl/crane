// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSION_CONTROLLER__CONTEXT_HPP_
#define CRANE_SESSION_CONTROLLER__CONTEXT_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <variant>
#include <vector>

namespace crane
{

struct PassAction
{
  uint8_t to_id;
  Point recieve_point;
};

struct ShootAction
{
  Point target;
};

struct DribbleAction
{
  Point target;
};

struct Context
{
  std::vector<std::variant<PassAction, ShootAction, DribbleAction>> actions;
};
}  // namespace crane
#endif  // CRANE_SESSION_CONTROLLER__CONTEXT_HPP_
