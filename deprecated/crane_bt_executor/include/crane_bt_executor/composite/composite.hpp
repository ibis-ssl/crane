// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
#ifndef CRANE_BT_EXECUTOR__COMPOSITE__COMPOSITE_HPP_
#define CRANE_BT_EXECUTOR__COMPOSITE__COMPOSITE_HPP_

// from standard library
#include <memory>  // std::shared_ptr
#include <vector>
// from custom header file
#include "crane_bt_executor/composite/component.hpp"

class Composite : public Component
{
public:
  std::vector<std::shared_ptr<Component>> children_;

public:
  Composite() {}

  void addChild(std::shared_ptr<Component> child) { children_.emplace_back(child); }

  void clearChild() { children_.clear(); }

  void serialize(ptree & my_tree) override;
};
#endif  // CRANE_BT_EXECUTOR__COMPOSITE__COMPOSITE_HPP_
