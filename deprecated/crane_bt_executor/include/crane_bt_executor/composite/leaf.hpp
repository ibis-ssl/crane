// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
#ifndef CRANE_BT_EXECUTOR__COMPOSITE__LEAF_HPP_
#define CRANE_BT_EXECUTOR__COMPOSITE__LEAF_HPP_

//  from custom header file
#include "crane_bt_executor/composite/component.hpp"

class Leaf : public Component
{
private:
public:
  Leaf()
  {
  }

  void serialize(ptree& my_tree) override;
};
#endif  // CRANE_BT_EXECUTOR__COMPOSITE__LEAF_HPP_
