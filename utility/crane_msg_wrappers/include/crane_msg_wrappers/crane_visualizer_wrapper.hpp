// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__CRANE_VISUALIZER_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__CRANE_VISUALIZER_WRAPPER_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_visualization_interfaces/msg/svg_layer_update.hpp>
#include <crane_visualization_interfaces/msg/svg_updates.hpp>
#include <memory>
#include <algorithm>
#include <range/v3/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
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

struct SvgBuilderBase
{
  std::shared_ptr<VisualizerMessageBuilder> builder;

  explicit SvgBuilderBase(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : builder(builder)
  {
  }

  virtual ~SvgBuilderBase() = default;

  [[nodiscard]] virtual auto getSvgString() const -> std::string = 0;

  auto build() const -> void { builder->add(getSvgString()); }
};

struct SvgCircleBuilder : public SvgBuilderBase
{
  Point circle_center;

  double circle_radius = 0.;

  std::string fill_color = "none";

  double fill_opacity = 1.;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  explicit SvgCircleBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    std::ostringstream oss;
    oss << "<circle cx=\"" << circle_center.x() * 1000. << "\" cy=\"" << -circle_center.y() * 1000.
        << "\" r=\"" << circle_radius * 1000. << "\" fill=\"" << fill_color << "\" fill-opacity=\""
        << fill_opacity << "\" stroke=\"" << stroke_color << "\" stroke-opacity=\""
        << stroke_opacity << "\" stroke-width=\"" << stroke_width << "\" />";
    return oss.str();
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

  [[nodiscard]] SvgCircleBuilder & fill(const std::string & color, double alpha = 1.0)
  {
    fill_color = color;
    fill_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto stroke(const std::string & color, double alpha = 1.0) -> SvgCircleBuilder &
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto strokeWidth(double width) -> SvgCircleBuilder &
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgLineBuilder : public SvgBuilderBase
{
  Point p1;
  Point p2;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  explicit SvgLineBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    std::ostringstream oss;
    oss << "<line x1=\"" << p1.x() * 1000. << "\" y1=\"" << -p1.y() * 1000. << "\" x2=\""
        << p2.x() * 1000. << "\" y2=\"" << -p2.y() * 1000. << "\" stroke=\"" << stroke_color
        << "\" stroke-opacity=\"" << stroke_opacity << "\" stroke-width=\"" << stroke_width
        << "\" />";
    return oss.str();
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

  [[nodiscard]] auto stroke(const std::string & color, double alpha = 1.0) -> SvgLineBuilder &
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto strokeWidth(double width) -> SvgLineBuilder &
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgRectBuilder : public SvgBuilderBase
{
  Point rect_top_left;

  Point rect_size;

  std::string fill_color = "none";

  double fill_opacity = 1.;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  explicit SvgRectBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    std::ostringstream oss;
    oss << "<rect x=\"" << rect_top_left.x() * 1000. << "\" y=\"" << -rect_top_left.y() * 1000.
        << "\" width=\"" << rect_size.x() * 1000. << "\" height=\"" << rect_size.y() * 1000.
        << "\" fill=\"" << fill_color << "\" fill-opacity=\"" << fill_opacity << "\" stroke=\""
        << stroke_color << "\" stroke-opacity=\"" << stroke_opacity << "\" stroke-width=\""
        << stroke_width << "\" />";
    return oss.str();
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

  [[nodiscard]] auto fill(const std::string & color, double alpha = 1.0) -> SvgRectBuilder &
  {
    fill_color = color;
    fill_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto stroke(const std::string & color, double alpha = 1.0) -> SvgRectBuilder &
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto strokeWidth(double width) -> SvgRectBuilder &
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgTextBuilder : public SvgBuilderBase
{
  Point text_position;

  std::string text_string;

  std::string fill_color = "white";

  double fill_opacity = 1.;

  double font_size = 100.0;

  bool view_box_position = false;

  std::string anchor = "start";

  explicit SvgTextBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    std::ostringstream oss;
    oss << "<text ";
    if (view_box_position) {
      oss << "x=\"" << text_position.x() << "%\" y=\"" << -text_position.y() << "%\" ";
    } else {
      oss << "x=\"" << text_position.x() * 1000. << "\" y=\"" << -text_position.y() * 1000.
          << "\" ";
    }
    oss << "fill=\"" << fill_color << "\" fill-opacity=\"" << fill_opacity << "\" font-size=\""
        << font_size << "\" text-anchor=\"" << anchor << "\">" << text_string << "</text>";
    return oss.str();
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

  [[nodiscard]] auto fill(const std::string & color, double alpha = 1.0) -> SvgTextBuilder &
  {
    fill_color = color;
    fill_opacity = alpha;
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

struct SvgPolyLineBuilder : public SvgBuilderBase
{
  std::vector<Point> points;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  explicit SvgPolyLineBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    std::ostringstream oss;
    oss << "<polyline points=\"";
    for (const auto & p : points) {
      oss << p.x() * 1000. << "," << -p.y() * 1000. << " ";
    }
    oss << "\" stroke=\"" << stroke_color << "\" stroke-width=\"" << stroke_width;
    if (stroke_opacity != 1.) {
      oss << "\" stroke-opacity=\"" << stroke_opacity;
    }
    oss << "\" fill=\"none\" />";
    return oss.str();
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

  [[nodiscard]] auto stroke(const std::string & color, double alpha = 1.0) -> SvgPolyLineBuilder &
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto strokeWidth(double width) -> SvgPolyLineBuilder &
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgPolygonBuilder : public SvgBuilderBase
{
  std::vector<Point> points;

  std::string fill_color = "none";

  double fill_opacity = 1.;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  explicit SvgPolygonBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    std::ostringstream oss;
    oss << "<polygon points=\"";
    for (const auto & p : points) {
      oss << p.x() * 1000. << "," << -p.y() * 1000. << " ";
    }
    oss << "\" fill=\"" << fill_color << "\" stroke=\"" << stroke_color << "\" stroke-width=\""
        << stroke_width << "\" />";
    return oss.str();
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

  [[nodiscard]] auto fill(const std::string & color, double alpha = 1.0) -> SvgPolygonBuilder &
  {
    fill_color = color;
    fill_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto stroke(const std::string & color, double alpha = 1.0) -> SvgPolygonBuilder &
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto strokeWidth(double width) -> SvgPolygonBuilder &
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgPathBuilder : public SvgBuilderBase
{
  std::string fill_color = "none";

  double fill_opacity = 1.;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  explicit SvgPathBuilder(const std::shared_ptr<VisualizerMessageBuilder> & builder)
  : SvgBuilderBase(builder)
  {
  }

  auto getSvgString() const -> std::string override
  {
    std::ostringstream oss;
    oss << "<path d=\"" << definition.path << "\" fill=\"" << fill_color << "\" stroke=\""
        << stroke_color << "\" stroke-width=\"" << stroke_width << "\" />";
    return oss.str();
  }

  [[nodiscard]] auto fill(const std::string & color, double alpha = 1.0) -> SvgPathBuilder &
  {
    fill_color = color;
    fill_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto stroke(const std::string & color, double alpha = 1.0) -> SvgPathBuilder &
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  [[nodiscard]] auto strokeWidth(double width) -> SvgPathBuilder &
  {
    stroke_width = width;
    return *this;
  }

  struct SvgPathDefinitionBuilder
  {
    std::string path;

    auto moveTo(double x, double y) -> SvgPathDefinitionBuilder &
    {
      path += " M" + std::to_string(x * 1000.) + "," + std::to_string(-y * 1000.);
      return *this;
    }

    auto moveTo(Point p) -> SvgPathDefinitionBuilder & { return moveTo(p.x(), p.y()); }

    SvgPathDefinitionBuilder & lineTo(double x, double y)
    {
      path += " L" + std::to_string(x * 1000.) + "," + std::to_string(-y * 1000.);
      return *this;
    }

    auto lineTo(Point p) -> SvgPathDefinitionBuilder & { return lineTo(p.x(), p.y()); }

    auto horizontalTo(double x) -> SvgPathDefinitionBuilder &
    {
      path += " H" + std::to_string(x * 1000.);
      return *this;
    }

    auto verticalTo(double y) -> SvgPathDefinitionBuilder &
    {
      path += " V" + std::to_string(-y * 1000.);
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
      path += " C" + std::to_string(x1 * 1000.) + "," + std::to_string(-y1 * 1000.) + " " +
              std::to_string(x2 * 1000.) + "," + std::to_string(-y2 * 1000.) + " " +
              std::to_string(x * 1000.) + "," + std::to_string(-y * 1000.);
      return *this;
    }

    auto cubicBezierTo(Point p1, Point p2, Point p) -> SvgPathDefinitionBuilder &
    {
      return cubicBezierTo(p1.x(), p1.y(), p2.x(), p2.y(), p.x(), p.y());
    }

    auto smoothCubicBezierTo(double x2, double y2, double x, double y) -> SvgPathDefinitionBuilder &
    {
      path += " S" + std::to_string(x2 * 1000.) + "," + std::to_string(-y2 * 1000.) + " " +
              std::to_string(x * 1000.) + "," + std::to_string(-y * 1000.);
      return *this;
    }

    auto smoothCubicBezierTo(Point p2, Point p) -> SvgPathDefinitionBuilder &
    {
      return smoothCubicBezierTo(p2.x(), p2.y(), p.x(), p.y());
    }

    auto quadraticBezierTo(double x1, double y1, double x, double y) -> SvgPathDefinitionBuilder &
    {
      path += " Q" + std::to_string(x1 * 1000.) + "," + std::to_string(-y1 * 1000.) + " " +
              std::to_string(x * 1000.) + "," + std::to_string(-y * 1000.);
      return *this;
    }

    auto quadraticBezierTo(Point p1, Point p) -> SvgPathDefinitionBuilder &
    {
      return quadraticBezierTo(p1.x(), p1.y(), p.x(), p.y());
    }

    auto smoothQuadraticBezierTo(double x, double y) -> SvgPathDefinitionBuilder &
    {
      path += " T" + std::to_string(x * 1000.) + "," + std::to_string(-y * 1000.);
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
      path += " A" + std::to_string(rx * 1000.) + "," + std::to_string(-ry * 1000.) + " " +
              std::to_string(x_axis_rotation) + " " + std::to_string(large_arc_flag) + "," +
              std::to_string(sweep_flag) + " " + std::to_string(x * 1000.) + "," +
              std::to_string(-y * 1000.);
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
      }else {
        // 指定レイヤーに対する未送信更新をローカルバッファから除去
        updates.erase(
          std::remove_if(
            updates.begin(), updates.end(),
            [&](const auto & u) { return u.layer == layer; }),
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
