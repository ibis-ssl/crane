// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__CRANE_VISUALIZER_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__CRANE_VISUALIZER_WRAPPER_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_visualization_interfaces/msg/svg_layer_update.hpp>
#include <crane_visualization_interfaces/msg/svg_updates.hpp>
#include <format>
#include <memory>
#include <range/v3/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
// 座標変換ユーティリティ（constexpr で高速化）
namespace SvgCoord
{
constexpr double SCALE = 1000.0;
constexpr double toSvgX(double x) { return x * SCALE; }
constexpr double toSvgY(double y) { return -y * SCALE; }
}  // namespace SvgCoord

// 基本色定数（よく使う色のみ）
namespace SvgColors
{
constexpr const char * White = "white";
constexpr const char * Black = "black";
constexpr const char * Red = "red";
constexpr const char * Green = "green";
constexpr const char * Blue = "blue";
constexpr const char * Yellow = "yellow";
constexpr const char * Cyan = "cyan";
constexpr const char * None = "none";
}  // namespace SvgColors

// 前方宣言
struct SvgCircleBuilder;
struct SvgPolygonBuilder;
struct SvgPolyLineBuilder;
struct SvgLineBuilder;
struct SvgRectBuilder;
struct SvgTextBuilder;
struct SvgPathBuilder;

struct VisualizerMessageBuilder : public std::enable_shared_from_this<VisualizerMessageBuilder>
{
  using SvgLayerUpdate = crane_visualization_interfaces::msg::SvgLayerUpdate;
  using SharedPtr = std::shared_ptr<VisualizerMessageBuilder>;

  std::string layer;
  std::string operation = "replace";  // default operation

  explicit VisualizerMessageBuilder(const std::string & layer) : layer(layer) {}

  auto flush() -> void;

  auto clear() -> void { message_buffer.clear(); }

  auto clearBuffer() -> void;

  std::vector<std::string> message_buffer;

  auto add(const std::string & svg_string) -> void { message_buffer.push_back(svg_string); }

  // Operation modifiers
  [[nodiscard]] auto asReplace() -> VisualizerMessageBuilder &
  {
    operation = "replace";
    return *this;
  }
  [[nodiscard]] auto asAppend() -> VisualizerMessageBuilder &
  {
    operation = "append";
    return *this;
  }
  [[nodiscard]] auto asClear() -> VisualizerMessageBuilder &
  {
    operation = "clear";
    return *this;
  }

  SvgCircleBuilder circle();

  auto line() -> SvgLineBuilder;

  auto polygon() -> SvgPolygonBuilder;

  auto polyline() -> SvgPolyLineBuilder;

  auto text() -> SvgTextBuilder;

  auto rect() -> SvgRectBuilder;

  auto path() -> SvgPathBuilder;
};

// スタイル属性の共通基底クラス（CRTP パターン）
template <typename Derived>
struct SvgStyleBuilder
{
  std::string fill_color = "none";
  double fill_opacity = 1.0;
  std::string stroke_color = "black";
  double stroke_opacity = 1.0;
  double stroke_width = 1.0;

  [[nodiscard]] auto fill(const std::string & color, double alpha = 1.0) -> Derived &
  {
    fill_color = color;
    fill_opacity = std::clamp(alpha, 0.0, 1.0);
    return static_cast<Derived &>(*this);
  }

  [[nodiscard]] auto stroke(const std::string & color, double alpha = 1.0) -> Derived &
  {
    stroke_color = color;
    stroke_opacity = std::clamp(alpha, 0.0, 1.0);
    return static_cast<Derived &>(*this);
  }

  [[nodiscard]] auto strokeWidth(double width) -> Derived &
  {
    stroke_width = width;
    return static_cast<Derived &>(*this);
  }
};

struct SvgBuilderBase
{
  std::shared_ptr<VisualizerMessageBuilder> builder;
  bool auto_build = false;  // RAII用フラグ
  bool built = false;       // 二重build防止

  explicit SvgBuilderBase(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : builder(builder)
  {
  }

  virtual ~SvgBuilderBase()
  {
    if (auto_build && !built) {
      build();
    }
  }

  [[nodiscard]] virtual auto getSvgString() const -> std::string = 0;

  auto build() -> void
  {
    if (!built) {
      builder->add(getSvgString());
      built = true;
    }
  }

  // RAII有効化（メソッドチェーンで使えるよう自身の参照を返す）
  template <typename Derived>
  [[nodiscard]] auto raii() -> Derived &
  {
    auto_build = true;
    return static_cast<Derived &>(*this);
  }
};

struct SvgCircleBuilder : public SvgBuilderBase, public SvgStyleBuilder<SvgCircleBuilder>
{
  Point circle_center;
  double circle_radius = 0.;

  explicit SvgCircleBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    using namespace SvgCoord;
    return std::format(
      "<circle cx=\"{:.3f}\" cy=\"{:.3f}\" r=\"{:.3f}\" "
      "fill=\"{}\" fill-opacity=\"{:.2f}\" stroke=\"{}\" "
      "stroke-opacity=\"{:.2f}\" stroke-width=\"{:.2f}\" />",
      toSvgX(circle_center.x()), toSvgY(circle_center.y()), circle_radius * SCALE, fill_color,
      fill_opacity, stroke_color, stroke_opacity, stroke_width);
  }

  [[nodiscard]] auto center(double x, double y) -> SvgCircleBuilder &
  {
    circle_center = Point(x, y);
    return *this;
  }

  [[nodiscard]] auto center(Point p) -> SvgCircleBuilder &
  {
    circle_center = p;
    return *this;
  }

  [[nodiscard]] auto radius(double radius) -> SvgCircleBuilder &
  {
    circle_radius = radius;
    return *this;
  }
};

struct SvgLineBuilder : public SvgBuilderBase, public SvgStyleBuilder<SvgLineBuilder>
{
  Point p1;
  Point p2;

  explicit SvgLineBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    using namespace SvgCoord;
    return std::format(
      "<line x1=\"{:.3f}\" y1=\"{:.3f}\" x2=\"{:.3f}\" y2=\"{:.3f}\" "
      "stroke=\"{}\" stroke-opacity=\"{:.2f}\" stroke-width=\"{:.2f}\" />",
      toSvgX(p1.x()), toSvgY(p1.y()), toSvgX(p2.x()), toSvgY(p2.y()), stroke_color,
      stroke_opacity, stroke_width);
  }

  [[nodiscard]] auto start(double x, double y) -> SvgLineBuilder & { return start(Point(x, y)); }

  [[nodiscard]] auto start(Point p) -> SvgLineBuilder &
  {
    p1 = p;
    return *this;
  }

  [[nodiscard]] auto end(double x, double y) -> SvgLineBuilder & { return end(Point(x, y)); }

  [[nodiscard]] auto end(Point p) -> SvgLineBuilder &
  {
    p2 = p;
    return *this;
  }

};

struct SvgRectBuilder : public SvgBuilderBase, public SvgStyleBuilder<SvgRectBuilder>
{
  Point rect_top_left;
  Point rect_size;

  explicit SvgRectBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    using namespace SvgCoord;
    return std::format(
      "<rect x=\"{:.3f}\" y=\"{:.3f}\" width=\"{:.3f}\" height=\"{:.3f}\" "
      "fill=\"{}\" fill-opacity=\"{:.2f}\" stroke=\"{}\" "
      "stroke-opacity=\"{:.2f}\" stroke-width=\"{:.2f}\" />",
      toSvgX(rect_top_left.x()), toSvgY(rect_top_left.y()), rect_size.x() * SCALE,
      rect_size.y() * SCALE, fill_color, fill_opacity, stroke_color, stroke_opacity,
      stroke_width);
  }

  [[nodiscard]] auto top_left(double x, double y) -> SvgRectBuilder &
  {
    rect_top_left = Point(x, y);
    return *this;
  }

  [[nodiscard]] auto top_left(Point p) -> SvgRectBuilder &
  {
    rect_top_left = p;
    return *this;
  }

  [[nodiscard]] auto size(double x, double y) -> SvgRectBuilder &
  {
    rect_size = Point(x, y);
    return *this;
  }

  [[nodiscard]] auto size(Point p) -> SvgRectBuilder &
  {
    rect_size = p;
    return *this;
  }

  [[nodiscard]] auto box(const Box & box) -> SvgRectBuilder &
  {
    rect_top_left = box.min_corner();
    rect_size = box.max_corner() - box.min_corner();
    return *this;
  }
};

struct SvgTextBuilder : public SvgBuilderBase, public SvgStyleBuilder<SvgTextBuilder>
{
  Point text_position;
  std::string text_string;
  double font_size = 100.0;
  bool view_box_position = false;
  std::string anchor = "start";

  explicit SvgTextBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
    // Textのデフォルト色はwhite
    fill_color = "white";
  }

  auto getSvgString() const -> std::string override
  {
    using namespace SvgCoord;
    if (view_box_position) {
      return std::format(
        "<text x=\"{}%\" y=\"{}%\" fill=\"{}\" fill-opacity=\"{:.2f}\" "
        "font-size=\"{:.2f}\" text-anchor=\"{}\">{}</text>",
        text_position.x(), -text_position.y(), fill_color, fill_opacity, font_size, anchor,
        text_string);
    } else {
      return std::format(
        "<text x=\"{:.3f}\" y=\"{:.3f}\" fill=\"{}\" fill-opacity=\"{:.2f}\" "
        "font-size=\"{:.2f}\" text-anchor=\"{}\">{}</text>",
        toSvgX(text_position.x()), toSvgY(text_position.y()), fill_color, fill_opacity, font_size,
        anchor, text_string);
    }
  }

  [[nodiscard]] auto position(double x, double y) -> SvgTextBuilder &
  {
    return position(Point(x, y));
  }

  [[nodiscard]] auto position(Point p) -> SvgTextBuilder &
  {
    view_box_position = false;
    text_position = p;
    return *this;
  }

  [[nodiscard]] auto viewBoxPosition(double x, double y) -> SvgTextBuilder &
  {
    return viewBoxPosition(Point(x, y));
  }

  [[nodiscard]] auto viewBoxPosition(Point p) -> SvgTextBuilder &
  {
    view_box_position = true;
    text_position = p;
    return *this;
  }

  [[nodiscard]] auto text(const std::string & text) -> SvgTextBuilder &
  {
    this->text_string = text;
    return *this;
  }

  [[nodiscard]] auto fontSize(double size) -> SvgTextBuilder &
  {
    font_size = size;
    return *this;
  }

  [[nodiscard]] auto textAnchor(const std::string & anchor) -> SvgTextBuilder &
  {
    this->anchor = anchor;
    return *this;
  }
};

struct SvgPolyLineBuilder : public SvgBuilderBase, public SvgStyleBuilder<SvgPolyLineBuilder>
{
  std::vector<Point> points;

  explicit SvgPolyLineBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    using namespace SvgCoord;
    std::ostringstream points_str;
    for (const auto & p : points) {
      points_str << std::format("{:.3f},{:.3f} ", toSvgX(p.x()), toSvgY(p.y()));
    }
    return std::format(
      "<polyline points=\"{}\" stroke=\"{}\" stroke-width=\"{:.2f}\" "
      "stroke-opacity=\"{:.2f}\" fill=\"none\" />",
      points_str.str(), stroke_color, stroke_width, stroke_opacity);
  }

  [[nodiscard]] auto addPoint(double x, double y) -> SvgPolyLineBuilder &
  {
    points.emplace_back(x, y);
    return *this;
  }

  [[nodiscard]] auto addPoint(Point p) -> SvgPolyLineBuilder &
  {
    points.push_back(p);
    return *this;
  }
};

struct SvgPolygonBuilder : public SvgBuilderBase, public SvgStyleBuilder<SvgPolygonBuilder>
{
  std::vector<Point> points;

  explicit SvgPolygonBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    using namespace SvgCoord;
    std::ostringstream points_str;
    for (const auto & p : points) {
      points_str << std::format("{:.3f},{:.3f} ", toSvgX(p.x()), toSvgY(p.y()));
    }
    return std::format(
      "<polygon points=\"{}\" fill=\"{}\" fill-opacity=\"{:.2f}\" "
      "stroke=\"{}\" stroke-opacity=\"{:.2f}\" stroke-width=\"{:.2f}\" />",
      points_str.str(), fill_color, fill_opacity, stroke_color, stroke_opacity, stroke_width);
  }

  [[nodiscard]] auto addPoint(double x, double y) -> SvgPolygonBuilder &
  {
    points.emplace_back(x, y);
    return *this;
  }

  [[nodiscard]] auto addPoint(Point p) -> SvgPolygonBuilder &
  {
    points.push_back(p);
    return *this;
  }
};

struct SvgPathBuilder : public SvgBuilderBase, public SvgStyleBuilder<SvgPathBuilder>
{
  explicit SvgPathBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    return std::format(
      "<path d=\"{}\" fill=\"{}\" fill-opacity=\"{:.2f}\" "
      "stroke=\"{}\" stroke-opacity=\"{:.2f}\" stroke-width=\"{:.2f}\" />",
      definition.path, fill_color, fill_opacity, stroke_color, stroke_opacity, stroke_width);
  }

  struct SvgPathDefinitionBuilder
  {
    std::string path;

    auto moveTo(double x, double y) -> SvgPathDefinitionBuilder &
    {
      using namespace SvgCoord;
      path += std::format(" M{:.3f},{:.3f}", toSvgX(x), toSvgY(y));
      return *this;
    }

    auto moveTo(Point p) -> SvgPathDefinitionBuilder & { return moveTo(p.x(), p.y()); }

    auto lineTo(double x, double y) -> SvgPathDefinitionBuilder &
    {
      using namespace SvgCoord;
      path += std::format(" L{:.3f},{:.3f}", toSvgX(x), toSvgY(y));
      return *this;
    }

    auto lineTo(Point p) -> SvgPathDefinitionBuilder & { return lineTo(p.x(), p.y()); }

    auto horizontalTo(double x) -> SvgPathDefinitionBuilder &
    {
      using namespace SvgCoord;
      path += std::format(" H{:.3f}", toSvgX(x));
      return *this;
    }

    auto verticalTo(double y) -> SvgPathDefinitionBuilder &
    {
      using namespace SvgCoord;
      path += std::format(" V{:.3f}", toSvgY(y));
      return *this;
    }

    auto closePath() -> SvgPathDefinitionBuilder &
    {
      path += " Z";
      return *this;
    }

    auto cubicBezierTo(double x1, double y1, double x2, double y2, double x, double y)
      -> SvgPathDefinitionBuilder &
    {
      using namespace SvgCoord;
      path += std::format(
        " C{:.3f},{:.3f} {:.3f},{:.3f} {:.3f},{:.3f}", toSvgX(x1), toSvgY(y1), toSvgX(x2),
        toSvgY(y2), toSvgX(x), toSvgY(y));
      return *this;
    }

    auto cubicBezierTo(Point p1, Point p2, Point p) -> SvgPathDefinitionBuilder &
    {
      return cubicBezierTo(p1.x(), p1.y(), p2.x(), p2.y(), p.x(), p.y());
    }

    auto smoothCubicBezierTo(double x2, double y2, double x, double y) -> SvgPathDefinitionBuilder &
    {
      using namespace SvgCoord;
      path += std::format(
        " S{:.3f},{:.3f} {:.3f},{:.3f}", toSvgX(x2), toSvgY(y2), toSvgX(x), toSvgY(y));
      return *this;
    }

    auto smoothCubicBezierTo(Point p2, Point p) -> SvgPathDefinitionBuilder &
    {
      return smoothCubicBezierTo(p2.x(), p2.y(), p.x(), p.y());
    }

    auto quadraticBezierTo(double x1, double y1, double x, double y) -> SvgPathDefinitionBuilder &
    {
      using namespace SvgCoord;
      path += std::format(
        " Q{:.3f},{:.3f} {:.3f},{:.3f}", toSvgX(x1), toSvgY(y1), toSvgX(x), toSvgY(y));
      return *this;
    }

    auto quadraticBezierTo(Point p1, Point p) -> SvgPathDefinitionBuilder &
    {
      return quadraticBezierTo(p1.x(), p1.y(), p.x(), p.y());
    }

    auto smoothQuadraticBezierTo(double x, double y) -> SvgPathDefinitionBuilder &
    {
      using namespace SvgCoord;
      path += std::format(" T{:.3f},{:.3f}", toSvgX(x), toSvgY(y));
      return *this;
    }

    auto smoothQuadraticBezierTo(Point p) -> SvgPathDefinitionBuilder &
    {
      return smoothQuadraticBezierTo(p.x(), p.y());
    }

    auto arcTo(
      double rx, double ry, double x_axis_rotation, bool large_arc_flag, bool sweep_flag, double x,
      double y) -> SvgPathDefinitionBuilder &
    {
      using namespace SvgCoord;
      path += std::format(
        " A{:.3f},{:.3f} {:.3f} {},{} {:.3f},{:.3f}", toSvgX(rx), toSvgY(ry), x_axis_rotation,
        static_cast<int>(large_arc_flag), static_cast<int>(sweep_flag), toSvgX(x), toSvgY(y));
      return *this;
    }

    auto arcTo(Point r, double x_axis_rotation, bool large_arc_flag, bool sweep_flag, Point p)
      -> SvgPathDefinitionBuilder &
    {
      return arcTo(r.x(), r.y(), x_axis_rotation, large_arc_flag, sweep_flag, p.x(), p.y());
    }
  } definition;
};

struct CraneVisualizerBuffer
{
  using SvgUpdates = crane_visualization_interfaces::msg::SvgUpdates;
  static inline std::unique_ptr<CraneVisualizerBuffer> buffer = nullptr;

  rclcpp::Publisher<SvgUpdates>::SharedPtr publisher;

  SvgUpdates message_buffer;

  static inline uint32_t s_epoch = 0;
  static inline uint32_t s_seq = 0;

  template <typename Node>
  CraneVisualizerBuffer(Node & node, const std::string topic)
  {
    publisher = node.template create_publisher<SvgUpdates>(topic, rclcpp::SensorDataQoS());
  }

  template <typename Node>
  static auto activate(Node & node, const std::string & topic = "/visualizer_svgs") -> void
  {
    if (not active()) {
      buffer = std::make_unique<CraneVisualizerBuffer>(node, topic);
    }
  }

  static auto deactivate() -> void
  {
    if (active()) {
      buffer.reset();
    }
  }

  static auto active() -> bool { return buffer != nullptr; }

  static auto publish() -> void
  {
    if (active()) {
      // Stamp and sequence
      buffer->message_buffer.header.stamp = rclcpp::Clock().now();
      buffer->message_buffer.epoch = s_epoch;
      buffer->message_buffer.seq = s_seq++;
      buffer->publisher->publish(buffer->message_buffer);
      buffer->message_buffer.updates.clear();
    }
  }

  static auto clear(const std::string & layer = "") -> void
  {
    if (CraneVisualizerBuffer::active()) {
      auto & updates = CraneVisualizerBuffer::buffer->message_buffer.updates;
      if (layer == "") {
        updates.clear();
      } else {
        // 指定レイヤーに対する未送信更新をローカルバッファから除去
        updates.erase(
          std::remove_if(
            updates.begin(), updates.end(), [&](const auto & u) { return u.layer == layer; }),
          updates.end());

        // 受信側の該当レイヤーを空にするため、空の更新を追加（publish() 時に送信される）
        crane_visualization_interfaces::msg::SvgLayerUpdate empty_layer;
        empty_layer.layer = layer;
        empty_layer.operation = "replace";  // 空レイヤーで置換 = 実質クリア
        // svg_primitives は空のまま
        updates.push_back(std::move(empty_layer));
      }
    }
  }

  static auto setEpoch(uint32_t epoch) -> void
  {
    s_epoch = epoch;
    s_seq = 0;
  }
};
}  // namespace crane
#endif  // CRANE_MSG_WRAPPERS__CRANE_VISUALIZER_WRAPPER_HPP_
