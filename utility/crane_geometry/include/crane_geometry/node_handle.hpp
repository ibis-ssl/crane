// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GEOMETRY__NODE_HANDLE_HPP_
#define CRANE_GEOMETRY__NODE_HANDLE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tuple>

namespace crane
{

namespace ros_node_interfaces_alias
{

using Base = rclcpp::node_interfaces::NodeBaseInterface;
using Clock = rclcpp::node_interfaces::NodeClockInterface;
using Graph = rclcpp::node_interfaces::NodeGraphInterface;
using Logging = rclcpp::node_interfaces::NodeLoggingInterface;
using TimeSource = rclcpp::node_interfaces::NodeTimeSourceInterface;
using Timers = rclcpp::node_interfaces::NodeTimersInterface;
using Topics = rclcpp::node_interfaces::NodeTopicsInterface;
using Waitables = rclcpp::node_interfaces::NodeWaitablesInterface;

}  // namespace ros_node_interfaces_alias

template <typename Interface>
inline std::shared_ptr<Interface> get_interface_from_node(rclcpp::Node & node);

// NodeBaseInterface
template <>
inline std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>
get_interface_from_node<rclcpp::node_interfaces::NodeBaseInterface>(rclcpp::Node & node)
{
  return node.get_node_base_interface();
}

// NodeClockInterface
template <>
inline std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface>
get_interface_from_node<rclcpp::node_interfaces::NodeClockInterface>(rclcpp::Node & node)
{
  return node.get_node_clock_interface();
}

// NodeGraphInterface
template <>
inline std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface>
get_interface_from_node<rclcpp::node_interfaces::NodeGraphInterface>(rclcpp::Node & node)
{
  return node.get_node_graph_interface();
}

// NodeLoggingInterface
template <>
inline std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface>
get_interface_from_node<rclcpp::node_interfaces::NodeLoggingInterface>(rclcpp::Node & node)
{
  return node.get_node_logging_interface();
}

// NodeTimeSourceInterface
template <>
inline std::shared_ptr<rclcpp::node_interfaces::NodeTimeSourceInterface>
get_interface_from_node<rclcpp::node_interfaces::NodeTimeSourceInterface>(rclcpp::Node & node)
{
  return node.get_node_time_source_interface();
}

// NodeTimersInterface
template <>
std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface>
get_interface_from_node<rclcpp::node_interfaces::NodeTimersInterface>(rclcpp::Node & node)
{
  return node.get_node_timers_interface();
}

// NodeTopicsInterface
template <>
inline std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>
get_interface_from_node<rclcpp::node_interfaces::NodeTopicsInterface>(rclcpp::Node & node)
{
  return node.get_node_topics_interface();
}

// NodeWaitablesInterface
template <>
inline std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface>
get_interface_from_node<rclcpp::node_interfaces::NodeWaitablesInterface>(rclcpp::Node & node)
{
  return node.get_node_waitables_interface();
}

template <typename... Interfaces>
class NodeHandle
{
public:
  explicit NodeHandle(std::shared_ptr<Interfaces>... interfaces)
  : interfaces_(std::make_tuple(interfaces...))
  {
  }

  explicit NodeHandle(rclcpp::Node & node)
  : NodeHandle(get_interface_from_node<Interfaces>(node)...)
  {
  }

  template <typename T>
  std::shared_ptr<T> get_interface()
  {
    return std::get<std::shared_ptr<T>>(interfaces_);
  }

private:
  std::tuple<std::shared_ptr<Interfaces>...> interfaces_;
};

}  // namespace crane

#endif  // CRANE_GEOMETRY__NODE_HANDLE_HPP_
