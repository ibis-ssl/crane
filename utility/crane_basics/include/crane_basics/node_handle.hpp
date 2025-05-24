// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_BASICS__NODE_HANDLE_HPP_
#define CRANE_BASICS__NODE_HANDLE_HPP_

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
inline auto get_interface_from_node(rclcpp::Node & node) -> std::shared_ptr<Interface>;

// NodeBaseInterface
template <>
inline auto get_interface_from_node<rclcpp::node_interfaces::NodeBaseInterface>(rclcpp::Node & node)
  -> std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>
{
  return node.get_node_base_interface();
}

// NodeClockInterface
template <>
inline auto get_interface_from_node<rclcpp::node_interfaces::NodeClockInterface>(
  rclcpp::Node & node) -> std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface>
{
  return node.get_node_clock_interface();
}

// NodeGraphInterface
template <>
inline auto get_interface_from_node<rclcpp::node_interfaces::NodeGraphInterface>(
  rclcpp::Node & node) -> std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface>
{
  return node.get_node_graph_interface();
}

// NodeLoggingInterface
template <>
inline auto get_interface_from_node<rclcpp::node_interfaces::NodeLoggingInterface>(
  rclcpp::Node & node) -> std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface>
{
  return node.get_node_logging_interface();
}

// NodeTimeSourceInterface
template <>
inline auto get_interface_from_node<rclcpp::node_interfaces::NodeTimeSourceInterface>(
  rclcpp::Node & node) -> std::shared_ptr<rclcpp::node_interfaces::NodeTimeSourceInterface>
{
  return node.get_node_time_source_interface();
}

// NodeTimersInterface
template <>
inline auto get_interface_from_node<rclcpp::node_interfaces::NodeTimersInterface>(
  rclcpp::Node & node) -> std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface>
{
  return node.get_node_timers_interface();
}

// NodeTopicsInterface
template <>
inline auto get_interface_from_node<rclcpp::node_interfaces::NodeTopicsInterface>(
  rclcpp::Node & node) -> inline std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface>
{
  return node.get_node_topics_interface();
}

// NodeWaitablesInterface
template <>
inline auto get_interface_from_node<rclcpp::node_interfaces::NodeWaitablesInterface>(
  rclcpp::Node & node) -> std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface>
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
  auto get_interface() -> std::shared_ptr<T>
  {
    return std::get<std::shared_ptr<T>>(interfaces_);
  }

private:
  std::tuple<std::shared_ptr<Interfaces>...> interfaces_;
};

}  // namespace crane

#endif  // CRANE_BASICS__NODE_HANDLE_HPP_
