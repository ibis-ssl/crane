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

  void addChild(std::shared_ptr<Component> child)
  {
    children_.emplace_back(child);
  }

  void clearChild()
  {
    children_.clear();
  }

  void serialize(ptree & my_tree) override;
};
#endif  // CRANE_BT_EXECUTOR__COMPOSITE__COMPOSITE_HPP_
