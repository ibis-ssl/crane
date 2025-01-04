// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__DEFENSE_FUNCTIONS_HPP_
#define CRANE_PLANNER_PLUGINS__DEFENSE_FUNCTIONS_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <optional>
#include <tuple>
#include <utility>

namespace crane
{
auto getPenaltyAreaCorners(
  double offset_x, double offset_y, const WorldModelWrapper::SharedPtr & world_model)
  -> std::tuple<Point, Point, Point, Point>;

auto getDefenseLinePointParameterThresholds(
  double offset_x, double offset_y, const WorldModelWrapper::SharedPtr & world_model)
  -> std::tuple<double, double, double>;

auto getDefenseLinePoint(double parameter, const WorldModelWrapper::SharedPtr & world_model)
  -> Point;

auto getDefenseLinePointParameter(
  const Segment & target_segment, const WorldModelWrapper::SharedPtr & world_model)
  -> std::optional<double>;

auto getPenaltyAreaCorners(
  double offset_x, double offset_y, const WorldModelWrapper::SharedPtr & world_model)
  -> std::tuple<Point, Point, Point, Point>;

auto getDefenseLinePointParameter(
  const Segment & target_segment, const WorldModelWrapper::SharedPtr & world_model)
  -> std::optional<double>;

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__DEFENSE_FUNCTIONS_HPP_
