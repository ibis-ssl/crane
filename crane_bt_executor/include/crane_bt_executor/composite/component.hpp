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
#ifndef CRANE_BT_EXECUTOR__COMPOSITE__COMPONENT_HPP_
#define CRANE_BT_EXECUTOR__COMPOSITE__COMPONENT_HPP_

//  from  standard library
#include <vector>
#include <string>
#include <functional>


class WorldModel;
namespace boost::property_tree
{
template<class Key, class Data, class KeyCompare>
class basic_ptree;

typedef basic_ptree<std::string, std::string, std::less<std::string>> ptree;
}
using boost::property_tree::ptree;

enum class Status
{
  NONE,
  FAILURE,
  SUCCESS,
  RUNNING,
};

class Component
{
public:
  std::string name;
  Status status = Status::RUNNING;

public:
  Component() {}

  virtual Status run(WorldModel & world_model, uint8_t my_id) = 0;

  virtual void serialize(ptree & my_tree) = 0;
};
#endif  // CRANE_BT_EXECUTOR__COMPOSITE__COMPONENT_HPP_
