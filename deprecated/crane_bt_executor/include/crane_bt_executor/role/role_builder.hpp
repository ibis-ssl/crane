// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__ROLE__ROLE_BUILDER_HPP_
#define CRANE_BT_EXECUTOR__ROLE__ROLE_BUILDER_HPP_

#include <cstdint>
#include <memory>

#include "crane_bt_executor/role/defender.hpp"
#include "crane_bt_executor/role/role_base.hpp"
#include "crane_bt_executor/role/role_id.hpp"
#include "crane_bt_executor/role/test/test_move.hpp"
#include "crane_bt_executor/utils/finite_state_machine.hpp"

class RoleBuilder
{
public:
  RoleBuilder()
  {
    //  エラー
    role_book_.fill([]() -> std::shared_ptr<RoleBase> {
      static_assert("Error : this role is not set");
      return nullptr;
    });

    registerRole<DefenderRole>(RoleID::DEFENDER);
    registerRole<TestMoveRole>(RoleID::TEST_MOVE);
  }

  std::shared_ptr<RoleBase> build(RoleID id) { return role_book_.at(static_cast<uint8_t>(id))(); }

private:
  template <typename RoleType>
  std::shared_ptr<RoleBase> build()
  {
    return std::make_shared<RoleType>();
  }
  template <typename RoleClass>
  void registerRole(RoleID id)
  {
    role_book_.at(static_cast<uint8_t>(id)) = std::bind(&RoleBuilder::build<RoleClass>, this);
  }

private:
  std::array<std::function<std::shared_ptr<RoleBase>()>, static_cast<uint8_t>(RoleID::ROLE_ID_NUM)>
    role_book_;
};
#endif  // CRANE_BT_EXECUTOR__ROLE__ROLE_BUILDER_HPP_
