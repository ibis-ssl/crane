// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER__VISUALIZATION_HELPERS_HPP_
#define CRANE_LOCAL_PLANNER__VISUALIZATION_HELPERS_HPP_

#include <crane_visualization_interfaces/crane_visualizer_wrapper.hpp>
#include <iomanip>
#include <sstream>

namespace crane
{
inline void drawRobotRadiusWithSpeed(
  const std::shared_ptr<VisualizerMessageBuilder> & visualizer, Point center, double radius,
  double speed, const std::string & color = "yellow", double circle_opacity = 0.2,
  double text_opacity = 0.5, double stroke_width = 10.0, double font_size = 50.0)
{
  // 半径の円を描画
  visualizer->circle()
    .center(center)
    .radius(radius)
    .stroke(color, circle_opacity)
    .strokeWidth(stroke_width)
    .build();

  // 速度テキストを円の上に描画
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << speed << "m/s";
  visualizer->text()
    .position(center + Vector2(0, radius + 0.07))
    .text(ss.str())
    .fontSize(font_size)
    .textAnchor("middle")
    .fill(color, text_opacity)
    .build();
}

}  // namespace crane

#endif  // CRANE_LOCAL_PLANNER__VISUALIZATION_HELPERS_HPP_
