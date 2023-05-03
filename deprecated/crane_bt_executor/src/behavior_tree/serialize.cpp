// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <boost/property_tree/ptree.hpp>
#include <utility>

#include "crane_bt_executor/behavior_tree/status_converter/status_converter.hpp"
#include "crane_bt_executor/composite/composite.hpp"
#include "crane_bt_executor/composite/leaf.hpp"

void StatusConverter::serialize(ptree & my_tree)
{
  my_tree.put("name", name_);
  my_tree.put("status", static_cast<uint8_t>(status_));

  ptree children;
  ptree base_info;
  base_->serialize(base_info);
  children.push_back(std::make_pair("", base_info));
  my_tree.add_child("children", children);
}

void Composite::serialize(ptree & my_tree)
{
  my_tree.put("name", name_);
  my_tree.put("status", static_cast<uint8_t>(status_));

  ptree children;
  for (auto child : this->children_) {
    ptree info;
    child->serialize(info);
    children.push_back(std::make_pair("", info));
  }

  my_tree.add_child("children", children);
}

void Leaf::serialize(ptree & my_tree)
{
  my_tree.put("name", name_);
  my_tree.put("status", static_cast<uint8_t>(status_));
}
