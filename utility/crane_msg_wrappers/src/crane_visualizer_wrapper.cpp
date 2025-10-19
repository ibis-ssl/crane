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
    update_msg.svg_primitives = message_buffer;
    CraneVisualizerBuffer::buffer->message_buffer.updates.push_back(update_msg);
    message_buffer.clear();
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
}  // namespace crane
