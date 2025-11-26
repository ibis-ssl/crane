// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__PLANNER_FACTORY_HPP_
#define CRANE_PLANNER_PLUGINS__PLANNER_FACTORY_HPP_

#include <crane_planner_plugins/planner_base.hpp>
#include <memory>
#include <string>
#include <vector>

namespace rclcpp
{
class Node;
}

namespace crane
{
struct WorldModelWrapper;

/// @brief プランナー名からプランナーインスタンスを生成
/// @param planner_name プランナー名
/// @param world_model ワールドモデルへの共有ポインタ
/// @param node ROS2ノード参照
/// @return 生成されたプランナーの共有ポインタ
/// @throw std::runtime_error 不明なプランナー名が指定された場合
auto generatePlanner(
  const std::string & planner_name, WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  -> PlannerBase::SharedPtr;

/// @brief 利用可能なプランナー名の一覧を取得
/// @return プランナー名のベクター
auto getAvailablePlannerNames() -> std::vector<std::string>;

}  // namespace crane

#endif  // CRANE_PLANNER_PLUGINS__PLANNER_FACTORY_HPP_
