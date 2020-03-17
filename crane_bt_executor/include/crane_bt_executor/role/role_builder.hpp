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

#ifndef CRANE_BT_EXECUTOR__ROLE__ROLE_BUILDER_HPP_
#define CRANE_BT_EXECUTOR__ROLE__ROLE_BUILDER_HPP_

#include <crane_bt_executor/role/defender.hpp>
#include <crane_bt_executor/role/role_base.hpp>
#include <crane_bt_executor/role/role_id.hpp>
#include <crane_bt_executor/utils/finite_state_machine.hpp>
#include <cstdint>
#include <memory>

class RoleBuilder{
public:
  RoleBuilder(){
    //  エラー
    role_book.fill([]()->std::shared_ptr<RoleBase>{
      static_assert("Error : this role is not set");
      return nullptr;
    });

    registerRole<DefenderRole>(RoleID::DEFENDER);
  }

  std::shared_ptr<RoleBase> build(RoleID id){
    return role_book.at(static_cast<uint8_t>(id))();
  }

private:
  template <typename RoleType>
  std::shared_ptr<RoleBase> build(){
    return std::make_shared<RoleType>();
  }
  template <typename RoleClass>
  void registerRole(RoleID id){
    role_book.at(static_cast<uint8_t>(id)) = std::bind(&RoleBuilder::build<RoleClass>, this);
  }

private:
  std::array<std::function<std::shared_ptr<RoleBase>()>,
      static_cast<uint8_t>(RoleID::ROLE_ID_NUM)> role_book;
};
#endif  // CRANE_BT_EXECUTOR__ROLE__ROLE_BUILDER_HPP_
