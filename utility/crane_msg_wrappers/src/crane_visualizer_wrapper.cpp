// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/crane_visualizer_wrapper.hpp"

namespace crane
{
auto VisualizerMessageBuilder::flush() -> void
{
  if (CraneVisualizerBuffer::active()) {
    SvgLayerUpdate update_msg;
    update_msg.layer = layer;
    update_msg.operation = operation;
    update_msg.svg_primitives = std::move(message_buffer);
    update_msg.duration = duration;
    CraneVisualizerBuffer::buffer->message_buffer.updates.push_back(update_msg);
  }
}

auto VisualizerMessageBuilder::clearBuffer() -> void
{
  clear();
  if (CraneVisualizerBuffer::active()) {
    CraneVisualizerBuffer::clear(layer);
  }
}

SvgCircleBuilder VisualizerMessageBuilder::circle() { return SvgCircleBuilder(shared_from_this()); }

SvgLineBuilder VisualizerMessageBuilder::line() { return SvgLineBuilder(shared_from_this()); }

SvgPolygonBuilder VisualizerMessageBuilder::polygon()
{
  return SvgPolygonBuilder(shared_from_this());
}

SvgPolyLineBuilder VisualizerMessageBuilder::polyline()
{
  return SvgPolyLineBuilder(shared_from_this());
}

SvgTextBuilder VisualizerMessageBuilder::text() { return SvgTextBuilder(shared_from_this()); }

SvgRectBuilder VisualizerMessageBuilder::rect() { return SvgRectBuilder(shared_from_this()); }

SvgPathBuilder VisualizerMessageBuilder::path() { return SvgPathBuilder(shared_from_this()); }

// 高レベル描画メソッドの実装
auto VisualizerMessageBuilder::arrow(
  Point start, Vector2 direction, double length, const std::string & color, double stroke_width,
  double arrowhead_length, double arrowhead_width) -> void
{
  Vector2 dir = direction.normalized();
  Point end = start + dir * length;

  // メインシャフト
  line().start(start).end(end).stroke(color).strokeWidth(stroke_width).build();

  // 矢じり（左右の羽根）
  Vector2 perp(-dir.y(), dir.x());
  Point left = end - dir * arrowhead_length + perp * arrowhead_width;
  Point right = end - dir * arrowhead_length - perp * arrowhead_width;
  line().start(end).end(left).stroke(color).strokeWidth(stroke_width).build();
  line().start(end).end(right).stroke(color).strokeWidth(stroke_width).build();
}

auto VisualizerMessageBuilder::velocityArrow(
  Point pos, Vector2 velocity, const std::string & color, double scale, double stroke_width) -> void
{
  double speed = std::sqrt(velocity.x() * velocity.x() + velocity.y() * velocity.y());
  if (speed < 0.01) return;  // 速度がほぼゼロなら描画しない

  double length = speed * scale;
  Vector2 dir = velocity / speed;
  Point end = pos + dir * length;

  // メインシャフト
  line().start(pos).end(end).stroke(color, 0.7).strokeWidth(stroke_width).build();

  // V字ヘッド（速度矢印用の小さめのヘッド）
  Vector2 perp(-dir.y(), dir.x());
  Point base_left = end + perp * 0.06 - dir * 0.03;
  Point base_right = end - perp * 0.06 - dir * 0.03;
  line().start(end).end(base_left).stroke(color, 0.5).strokeWidth(stroke_width).build();
  line().start(end).end(base_right).stroke(color, 0.5).strokeWidth(stroke_width).build();
}

auto VisualizerMessageBuilder::labeledCircle(
  Point center, double radius, const std::string & label, const std::string & circle_color,
  const std::string & text_color, double circle_stroke_width, double text_font_size) -> void
{
  // 円を描画
  circle()
    .center(center)
    .radius(radius)
    .stroke(circle_color)
    .strokeWidth(circle_stroke_width)
    .build();

  // ラベルを描画（円の下に配置）
  text()
    .position(center + Vector2(0.0, -radius - 0.1))
    .text(label)
    .fill(text_color)
    .fontSize(text_font_size)
    .textAnchor("middle")
    .build();
}

auto VisualizerMessageBuilder::arc(
  Point center, double radius, double start_angle, double end_angle, const std::string & color,
  double stroke_width, int steps) -> void
{
  auto arc_builder = polyline().stroke(color).strokeWidth(stroke_width);
  for (int i = 0; i <= steps; ++i) {
    double t = static_cast<double>(i) / steps;
    double angle = start_angle + (end_angle - start_angle) * t;
    Point p = center + Vector2(std::cos(angle), std::sin(angle)) * radius;
    (void)arc_builder.addPoint(p);  // nodiscard警告を抑制
  }
  arc_builder.build();
}

auto VisualizerMessageBuilder::doubleCircle(
  Point center, double inner_radius, double outer_radius, const std::string & inner_color,
  const std::string & outer_color, double inner_stroke_width, double outer_stroke_width) -> void
{
  // 外側の円（背景）
  circle()
    .center(center)
    .radius(outer_radius)
    .stroke(outer_color)
    .strokeWidth(outer_stroke_width)
    .build();

  // 内側の円（inner_stroke_widthは内側の円には枠線がないため使用しない）
  (void)inner_stroke_width;  // 未使用パラメータ警告を抑制（将来の拡張用に残す）
  circle().center(center).radius(inner_radius).fill(inner_color, 0.25).stroke("none").build();
}

auto VisualizerMessageBuilder::rectangle(
  Point top_left, Point bottom_right, const std::string & color, double stroke_width) -> void
{
  // 4本の線で矩形を描画
  Point top_right(bottom_right.x(), top_left.y());
  Point bottom_left(top_left.x(), bottom_right.y());

  line().start(top_left).end(top_right).stroke(color).strokeWidth(stroke_width).build();
  line().start(top_right).end(bottom_right).stroke(color).strokeWidth(stroke_width).build();
  line().start(bottom_right).end(bottom_left).stroke(color).strokeWidth(stroke_width).build();
  line().start(bottom_left).end(top_left).stroke(color).strokeWidth(stroke_width).build();
}

// Quick描画メソッドの実装
auto VisualizerMessageBuilder::drawLine(
  Point start, Point end, const std::string & color, double stroke_width, double opacity) -> void
{
  line().start(start).end(end).stroke(color, opacity).strokeWidth(stroke_width).build();
}

auto VisualizerMessageBuilder::drawCircle(
  Point center, double radius, const std::string & color, double stroke_width, double opacity)
  -> void
{
  circle().center(center).radius(radius).stroke(color, opacity).strokeWidth(stroke_width).build();
}

auto VisualizerMessageBuilder::drawFilledCircle(
  Point center, double radius, const std::string & fill_color, double opacity) -> void
{
  circle().center(center).radius(radius).fill(fill_color, opacity).stroke("none").build();
}

auto VisualizerMessageBuilder::drawStyledCircle(
  Point center, double radius, const std::string & fill_color, double fill_opacity,
  const std::string & stroke_color, double stroke_opacity, double stroke_width) -> void
{
  circle()
    .center(center)
    .radius(radius)
    .fill(fill_color, fill_opacity)
    .stroke(stroke_color, stroke_opacity)
    .strokeWidth(stroke_width)
    .build();
}

auto VisualizerMessageBuilder::drawPolyline(
  const std::vector<Point> & points, const std::string & color, double opacity, double stroke_width)
  -> void
{
  polyline().setPoints(points).stroke(color, opacity).strokeWidth(stroke_width).build();
}

auto VisualizerMessageBuilder::drawText(
  Point position, const std::string & text_str, const std::string & color, double font_size,
  const std::string & anchor) -> void
{
  text()
    .position(position)
    .text(text_str)
    .fill(color)
    .fontSize(font_size)
    .textAnchor(anchor)
    .build();
}

// フィールド描画専用の便利関数の実装
auto VisualizerMessageBuilder::drawFieldLine(
  Point p1, Point p2, const std::string & color, double stroke_width) -> void
{
  line().start(p1).end(p2).stroke(color).strokeWidth(stroke_width).build();
}

auto VisualizerMessageBuilder::drawFieldRect(
  Point corner1, Point corner2, const std::string & color, double stroke_width) -> void
{
  Point top_left(std::min(corner1.x(), corner2.x()), std::max(corner1.y(), corner2.y()));
  Point bottom_right(std::max(corner1.x(), corner2.x()), std::min(corner1.y(), corner2.y()));
  rectangle(top_left, bottom_right, color, stroke_width);
}

auto VisualizerMessageBuilder::drawGoal(
  Point back_center, double width, double depth, const std::string & color, double stroke_width)
  -> void
{
  // U字型のゴールを描画（奥の壁と左右の壁）
  double half_width = width / 2.0;
  Point top_left = back_center + Vector2(depth, -half_width);
  Point top_right = back_center + Vector2(depth, half_width);
  Point bottom_left = back_center + Vector2(0.0, -half_width);
  Point bottom_right = back_center + Vector2(0.0, half_width);

  // 3本の線で描画（奥、左、右）
  line().start(top_left).end(top_right).stroke(color).strokeWidth(stroke_width).build();
  line().start(top_left).end(bottom_left).stroke(color).strokeWidth(stroke_width).build();
  line().start(top_right).end(bottom_right).stroke(color).strokeWidth(stroke_width).build();
}

// テキスト表示のプリセット関数の実装
auto VisualizerMessageBuilder::drawDebugLabel(
  Point robot_pos, const std::string & label, const std::string & color, double offset_x,
  double offset_y) -> void
{
  text()
    .position(robot_pos.x() + offset_x, robot_pos.y() + offset_y)
    .text(label)
    .fill(color)
    .fontSize(100)
    .build();
}

auto VisualizerMessageBuilder::drawCenteredLabel(
  Point pos, const std::string & label, const std::string & color, double font_size) -> void
{
  text().position(pos).text(label).fill(color).fontSize(font_size).textAnchor("middle").build();
}

// ロボット描画の便利関数の実装
auto VisualizerMessageBuilder::drawRobot(
  Point pos, double theta, const std::string & fill_color, double fill_opacity,
  const std::string & stroke_color, double stroke_opacity, double stroke_width, double radius,
  double center_to_dribbler) -> void
{
  // ロボット形状の計算（SvgRobotBuilderと同じロジック）
  double corner_angle = std::acos(center_to_dribbler / radius);
  auto botRightX = [&](double orientation) {
    return radius * std::cos(orientation + corner_angle);
  };
  auto botRightY = [&](double orientation) {
    return radius * std::sin(orientation + corner_angle);
  };
  auto botLeftX = [&](double orientation) { return radius * std::cos(orientation - corner_angle); };
  auto botLeftY = [&](double orientation) { return radius * std::sin(orientation - corner_angle); };

  using SvgCoord::SCALE;
  double right_x = (pos.x() + botRightX(theta)) * SCALE;
  double right_y = (pos.y() + botRightY(theta)) * -SCALE;
  double left_x = (pos.x() + botLeftX(theta)) * SCALE;
  double left_y = (pos.y() + botLeftY(theta)) * -SCALE;

  std::string svg_path = std::format(
    "<path d=\"M {:.3f} {:.3f} A {:.3f} {:.3f} 0 1 0 {:.3f} {:.3f} Z\" "
    "fill=\"{}\" fill-opacity=\"{:.2f}\" stroke=\"{}\" stroke-opacity=\"{:.2f}\" "
    "stroke-width=\"{:.2f}\"/>",
    right_x, right_y, radius * SCALE, radius * SCALE, left_x, left_y, fill_color, fill_opacity,
    stroke_color, stroke_opacity, stroke_width);

  add(svg_path);
}

auto VisualizerMessageBuilder::drawRobotWithID(
  Point pos, double theta, int id, const std::string & fill_color, double fill_opacity,
  const std::string & stroke_color, double stroke_opacity, double stroke_width, double id_font_size,
  const std::string & id_color, double id_offset_x, double id_offset_y) -> void
{
  // ロボット本体を描画
  drawRobot(pos, theta, fill_color, fill_opacity, stroke_color, stroke_opacity, stroke_width);

  // IDを描画
  text()
    .text(std::to_string(id))
    .position(pos.x() + id_offset_x, pos.y() + id_offset_y)
    .fontSize(id_font_size)
    .fill(id_color)
    .build();
}

// 軌跡描画の便利関数の実装
auto VisualizerMessageBuilder::drawTrajectory(
  const std::vector<Point> & points, const std::string & color, double base_opacity,
  int sampling_interval, double stroke_width) -> void
{
  if (points.empty()) return;

  auto polyline_builder = polyline().stroke(color, base_opacity).strokeWidth(stroke_width);
  for (size_t i = 0; i < points.size(); i += sampling_interval) {
    polyline_builder = polyline_builder.addPoint(points[i]);
  }
  // 最後のポイントが含まれていない場合は追加
  if ((points.size() - 1) % sampling_interval != 0) {
    polyline_builder = polyline_builder.addPoint(points.back());
  }
  polyline_builder.build();
}

auto VisualizerMessageBuilder::drawFadingTrajectory(
  const std::vector<Point> & points, const std::string & color, int segments, double stroke_width,
  int sampling_interval) -> void
{
  if (points.empty() || segments <= 0) return;

  // セグメント数を調整（ポイント数より多い場合はポイント数に合わせる）
  int actual_segments = std::min(segments, static_cast<int>(points.size()));

  for (int i = 0; i < actual_segments; ++i) {
    int start = static_cast<int>((points.size() / static_cast<double>(actual_segments)) * i);
    int end = static_cast<int>((points.size() / static_cast<double>(actual_segments)) * (i + 1));

    // セグメント内のポイントを描画
    auto polyline_builder = polyline();
    for (int index = start; index < end; index += sampling_interval) {
      polyline_builder = polyline_builder.addPoint(points[index]);
    }
    // 最後のセグメントでない場合、終点を追加
    if (i != actual_segments - 1 && end < static_cast<int>(points.size())) {
      polyline_builder = polyline_builder.addPoint(points[end]);
    }

    // グラデーション透明度（古い軌跡ほど薄く）
    double opacity = 0.5 * start / static_cast<double>(points.size());
    polyline_builder.stroke(color, opacity).strokeWidth(stroke_width).build();
  }
}
}  // namespace crane
