// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_TACTICS__DEFENSE_FUNCTIONS_HPP_
#define CRANE_TACTICS__DEFENSE_FUNCTIONS_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <optional>
#include <tuple>
#include <utility>

namespace crane
{
// 防御関連の定数
// ペナルティエリア境界からのオフセット
constexpr double DEFENSE_OFFSET_X = 0.2;
constexpr double DEFENSE_OFFSET_Y = 0.2;

// 防御配置の間隔
constexpr double DEFENSE_ARC_INTERVAL = 0.5;
constexpr double DEFENSE_LINE_INTERVAL = 0.2;
constexpr double DEFENSE_RADIUS_OFFSET = 0.5;

auto getDefenseLinePointParameterThresholds(
  double offset_x, double offset_y, const WorldModelWrapper::SharedPtr & world_model)
  -> std::tuple<double, double, double>;

auto getDefenseLinePoint(double parameter, const WorldModelWrapper::SharedPtr & world_model)
  -> Point;

auto getDefenseLinePointParameter(
  const Segment & target_segment, const WorldModelWrapper::SharedPtr & world_model)
  -> std::optional<double>;

// 円弧状の防御ポイントを計算
auto getDefenseArcPoints(
  int robot_num, const Segment & ball_line, const WorldModelWrapper::SharedPtr & world_model)
  -> std::vector<Point>;

// 防御ライン上のポイントを計算
auto getDefenseLinePoints(
  int robot_num, const Segment & ball_line, const WorldModelWrapper::SharedPtr & world_model,
  bool is_open_center = false, std::optional<double> defense_parameter = std::nullopt)
  -> std::vector<Point>;
}  // namespace crane
#endif  // CRANE_TACTICS__DEFENSE_FUNCTIONS_HPP_
