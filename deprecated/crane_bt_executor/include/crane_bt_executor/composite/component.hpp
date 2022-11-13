// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
#ifndef CRANE_BT_EXECUTOR__COMPOSITE__COMPONENT_HPP_
#define CRANE_BT_EXECUTOR__COMPOSITE__COMPONENT_HPP_

//  from  standard library
#include <vector>
#include <string>
#include <memory>
#include <functional>

class WorldModelWrapper;
class RobotIO;
namespace boost::property_tree
{
template <class Key, class Data, class KeyCompare>
class basic_ptree;

typedef basic_ptree<std::string, std::string, std::less<std::string>> ptree;
}  // namespace boost::property_tree
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
  std::string name_;
  Status status_ = Status::RUNNING;

public:
  Component()
  {
  }

  virtual Status run(WorldModelWrapper::SharedPtr world_model, RobotIO robot) = 0;

  virtual void serialize(ptree& my_tree) = 0;
};
#endif  // CRANE_BT_EXECUTOR__COMPOSITE__COMPONENT_HPP_
