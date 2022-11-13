// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_FIELD_ANALYZER__ROLES__ROLE_BASE_HPP_
#define CRANE_FIELD_ANALYZER__ROLES__ROLE_BASE_HPP_

#include "crane_msg_wrappers/play_situation_wrapper.hpp"
#include "crane_msg_wrappers/role_score_wrapper.hpp"

namespace crane
{
class RoleBase
{
public:
  RoleBase()
  {
  }
  virtual bool isAvailable(const PlaySituationWrapper& msg) const = 0;
  virtual void calcRoleScore(const PlaySituationWrapper& play_situation, RoleScoreWrapper& role_score) = 0;
};
}  // namespace crane
#endif  // CRANE_FIELD_ANALYZER__ROLES__ROLE_BASE_HPP_
