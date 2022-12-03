// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BT_EXECUTOR__BEHAVIOR_TREE__STATUS_CONVERTER__STATUS_CONVERTER_HPP_
#define CRANE_BT_EXECUTOR__BEHAVIOR_TREE__STATUS_CONVERTER__STATUS_CONVERTER_HPP_

#include <memory>
#include <string>
#include "crane_bt_executor/composite/component.hpp"

class StatusConverter : public Component
{
protected:
  std::shared_ptr<Component> base_;

public:
  explicit StatusConverter(std::string name, std::shared_ptr<Component> base) : Component(), base_(base)
  {
    this->name_ = name;
  }

  void serialize(ptree& my_tree) override;
};
#endif  // CRANE_BT_EXECUTOR__BEHAVIOR_TREE__STATUS_CONVERTER__STATUS_CONVERTER_HPP_
