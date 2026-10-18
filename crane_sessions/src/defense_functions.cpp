// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <crane_sessions/defense_functions.hpp>
#include <tuple>

namespace crane
{
namespace
{
auto getDefenseLinePointParameterThresholds(
  double offset_x, double offset_y, const WorldModelWrapper::SharedPtr & world_model)
  -> std::tuple<double, double, double>
{
  const double threshold1 = world_model->penaltyAreaSize().x() + offset_x + 0.5;
  // p2 -> p3: world_model->penaltyAreaSize().y() + OFFSET_Y * 2
  const double threshold2 = world_model->penaltyAreaSize().y() + offset_y * 2 + threshold1;
  // p3 -> p4: world_model->penaltyAreaSize().x() + OFFSET_X
  const double threshold3 = world_model->penaltyAreaSize().x() + offset_x + threshold2 + 0.5;
  return {threshold1, threshold2, threshold3};
}
}  // namespace

auto getDefenseLinePoint(double parameter, const WorldModelWrapper::SharedPtr & world_model)
  -> Point
{
  auto [p1, p2, p3, p4] = world_model->getPenaltyAreaCorners(DEFENSE_OFFSET_X, DEFENSE_OFFSET_Y);

  const auto [threshold1, threshold2, threshold3] =
    getDefenseLinePointParameterThresholds(DEFENSE_OFFSET_X, DEFENSE_OFFSET_Y, world_model);

  if (parameter >= 0. && parameter < threshold1) {
    return p1 + (p2 - p1).normalized() * parameter;
  } else if (parameter < threshold2) {
    return p2 + (p3 - p2).normalized() * (parameter - threshold1);
  } else if (parameter < threshold3) {
    return p3 + (p4 - p3).normalized() * (parameter - threshold2);
  } else {
    if (parameter < 0.) {
      return p1 + (p2 - p1).normalized() * parameter;
    } else if (parameter > threshold3) {
      return p3 + (p4 - p3).normalized() * (parameter - threshold2);
    } else {
      std::stringstream what;
      what << "Invalid parameter range for getDefenseLinePoint: " << parameter;
      what << "with thresholds: " << threshold1 << ", " << threshold2 << ", " << threshold3;
      throw std::runtime_error(what.str());
    }
  }
}

auto getDefenseLinePointParameter(
  const Segment & target_segment, const WorldModelWrapper::SharedPtr & world_model)
  -> std::optional<double>
{
  auto [p1, p2, p3, p4] = world_model->getPenaltyAreaCorners(DEFENSE_OFFSET_X, DEFENSE_OFFSET_Y);

  const auto [threshold1, threshold2, threshold3_unused] =
    getDefenseLinePointParameterThresholds(DEFENSE_OFFSET_X, DEFENSE_OFFSET_Y, world_model);

  if (
    auto intersections = getIntersections(Segment{p1, p2}, target_segment);
    not intersections.empty()) {
    return std::abs(intersections[0].x() - p1.x());
  } else if (
    intersections = getIntersections(Segment{p2, p3}, target_segment); not intersections.empty()) {
    return std::abs(intersections[0].y() - p2.y()) + threshold1;
  } else if (
    intersections = getIntersections(Segment{p3, p4}, target_segment); not intersections.empty()) {
    return std::abs(intersections[0].x() - p3.x()) + threshold2;
  } else {
    return std::nullopt;
  }
}

auto getDefenseLinePoints(
  int robot_num, const Segment & ball_line, const WorldModelWrapper::SharedPtr & world_model,
  bool is_open_center, std::optional<double> defense_parameter_opt) -> std::vector<Point>
{
  std::vector<Point> defense_points;

  // defense_parameterが指定されていない場合は計算
  auto defense_parameter_value = defense_parameter_opt.value_or(
    getDefenseLinePointParameter(ball_line, world_model).value_or(-1.0));

  if (defense_parameter_value < 0.0) {
    return defense_points;
  }

  double upper_parameter = defense_parameter_value;
  double lower_parameter = defense_parameter_value;

  auto add_parameter = [&](double parameter) -> bool {
    auto [threshold1, threshold2, threshold3] =
      getDefenseLinePointParameterThresholds(DEFENSE_OFFSET_X, DEFENSE_OFFSET_Y, world_model);
    if (parameter < 0. || parameter > threshold3) {
      return false;
    } else {
      if (upper_parameter < parameter) {
        upper_parameter = parameter;
      }
      if (lower_parameter > parameter) {
        lower_parameter = parameter;
      }
      defense_points.push_back(getDefenseLinePoint(parameter, world_model));
      return true;
    }
  };

  // 1台目
  if (not is_open_center) {
    add_parameter(defense_parameter_value);
  }

  // is_open_centerがtrueのときは両脇から配置開始する
  const int remaining_robot_num = is_open_center ? robot_num : robot_num - 1;
  // 中央の開け具合を計算する。前進守備するとき(ゴールにボールが近いとき)は開けない
  auto open_center_ratio_opt = world_model->getForwardDefenseRatio(ball_line);
  double open_center_interval = 0.0;
  if (not open_center_ratio_opt) {
    open_center_interval = DEFENSE_LINE_INTERVAL;
  } else {
    open_center_interval = (1.0 - (*open_center_ratio_opt)) * DEFENSE_LINE_INTERVAL;
  }

  // 2台目以降
  for (int i = 0; i < remaining_robot_num; i++) {
    if (is_open_center && i < 2) {
      // 中央を開けるとき
      if (i == 0) {
        if (not add_parameter(upper_parameter + open_center_interval)) {
          add_parameter(lower_parameter - open_center_interval);
        }
      } else if (i == 1) {
        if (not add_parameter(lower_parameter - open_center_interval)) {
          add_parameter(upper_parameter + open_center_interval);
        }
      }
    } else if (i % 2 == 0) {
      // upper側に追加
      if (not add_parameter(upper_parameter + DEFENSE_LINE_INTERVAL)) {
        // だめならlower側
        add_parameter(lower_parameter - DEFENSE_LINE_INTERVAL);
      }
    } else {
      // lower側に追加
      if (not add_parameter(lower_parameter - DEFENSE_LINE_INTERVAL)) {
        // だめならupper側
        add_parameter(upper_parameter + DEFENSE_LINE_INTERVAL);
      }
    }
  }

  return defense_points;
}
}  // namespace crane
