// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_UTILS__PARAMETER_HPP_
#define CRANE_UTILS__PARAMETER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{

/**
 * @brief パラメータが未宣言なら宣言し、設定値（またはデフォルト値）を取得する
 *
 * @tparam T パラメータ型
 * @param node ROS 2 ノード参照
 * @param name パラメータ名
 * @param default_value デフォルト値
 * @return T 取得したパラメータ値
 */
template <typename T>
auto declare_parameter_if_not_declared(
  rclcpp::Node & node, const std::string & name, const T & default_value) -> T
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).template get_value<T>();
  }
  return node.declare_parameter(name, default_value);
}

/**
 * @brief 文字列リテラル (const char*) 用のオーバーロード
 *
 * @param node ROS 2 ノード参照
 * @param name パラメータ名
 * @param default_value デフォルト文字列
 * @return std::string 取得したパラメータ文字列
 */
inline auto declare_parameter_if_not_declared(
  rclcpp::Node & node, const std::string & name, const char * default_value) -> std::string
{
  return declare_parameter_if_not_declared(node, name, std::string(default_value));
}

/**
 * @brief パラメータが未宣言なら宣言し、設定値（またはデフォルト値）を取得する
 */
template <typename T>
auto get_or_declare_parameter(
  rclcpp::Node & node, const std::string & name, const T & default_value) -> T
{
  return declare_parameter_if_not_declared(node, name, default_value);
}

inline auto get_or_declare_parameter(
  rclcpp::Node & node, const std::string & name, const char * default_value) -> std::string
{
  return declare_parameter_if_not_declared(node, name, default_value);
}

/**
 * @brief 変数の参照を渡し、その初期値をデフォルト値として未宣言なら宣言し、取得値を代入する
 *
 * @tparam T パラメータ型
 * @param node ROS 2 ノード参照
 * @param name パラメータ名
 * @param value デフォルト値として使われ、取得値が上書き代入される参照
 */
template <typename T>
auto get_or_declare_parameter(rclcpp::Node & node, const std::string & name, T & value) -> T &
{
  value = declare_parameter_if_not_declared(node, name, static_cast<const T &>(value));
  return value;
}

/**
 * @brief Node ポインタ (this) から直接呼べるオーバーロード
 */
template <typename T>
auto get_or_declare_parameter(
  rclcpp::Node * node, const std::string & name, const T & default_value) -> T
{
  return get_or_declare_parameter(*node, name, default_value);
}

inline auto get_or_declare_parameter(
  rclcpp::Node * node, const std::string & name, const char * default_value) -> std::string
{
  return get_or_declare_parameter(*node, name, default_value);
}

template <typename T>
auto get_or_declare_parameter(rclcpp::Node * node, const std::string & name, T & value) -> T &
{
  return get_or_declare_parameter(*node, name, value);
}

}  // namespace crane

#endif  // CRANE_UTILS__PARAMETER_HPP_
