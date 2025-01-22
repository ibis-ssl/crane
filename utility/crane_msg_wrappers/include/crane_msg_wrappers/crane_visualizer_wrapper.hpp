// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_MSG_WRAPPERS__CRANE_VISUALIZER_WRAPPER_HPP_
#define CRANE_MSG_WRAPPERS__CRANE_VISUALIZER_WRAPPER_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_visualization_interfaces/msg/svg_primitive_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace crane
{
struct SvgCircleBuilder
{
  Point circle_center;

  double circle_radius;

  std::string fill_color = "none";

  double fill_opacity = 1.;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  SvgCircleBuilder() {}

  std::string getSvgString() const
  {
    std::ostringstream oss;
    oss << "<circle cx=\"" << circle_center.x() * 1000. << "\" cy=\"" << circle_center.y() * 1000.
        << "\" r=\"" << circle_radius << "\" fill=\"" << fill_color << "\" stroke=\""
        << stroke_color << "\" stroke-width=\"" << stroke_width << "\" />";
    return oss.str();
  }

  SvgCircleBuilder & center(double x, double y)
  {
    circle_center = Point(x, y);
    return *this;
  }

  SvgCircleBuilder & center(Point p)
  {
    circle_center = p;
    return *this;
  }

  SvgCircleBuilder & radius(double radius)
  {
    circle_radius = radius;
    return *this;
  }

  SvgCircleBuilder & fill(const std::string & color, double alpha = 1.0)
  {
    fill_color = color;
    fill_opacity = alpha;
    return *this;
  }

  SvgCircleBuilder & stroke(const std::string & color, double alpha = 1.0)
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  SvgCircleBuilder & strokeWidth(double width)
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgLineBuilder
{
  Point p1;
  Point p2;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  SvgLineBuilder() {}

  std::string getSvgString() const
  {
    std::ostringstream oss;
    oss << "<line x1=\"" << p1.x() * 1000. << "\" y1=\"" << p1.y() * 1000. << "\" x2=\""
        << p2.x() * 1000. << "\" y2=\"" << p2.y() * 1000. << "\" stroke=\"" << stroke_color
        << "\" stroke-width=\"" << stroke_width << "\" />";
    return oss.str();
  }

  SvgLineBuilder & start(double x, double y) { return start(Point(x, y)); }

  SvgLineBuilder & start(Point p)
  {
    p1 = p;
    return *this;
  }

  SvgLineBuilder & end(double x, double y) { return end(Point(x, y)); }

  SvgLineBuilder & end(Point p)
  {
    p2 = p;
    return *this;
  }

  SvgLineBuilder & stroke(const std::string & color, double alpha = 1.0)
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  SvgLineBuilder & strokeWidth(double width)
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgRectBuilder
{
  Point rect_top_left;

  Point rect_size;

  std::string fill_color = "none";

  double fill_opacity = 1.;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  SvgRectBuilder() {}

  std::string getSvgString() const
  {
    std::ostringstream oss;
    oss << "<rect x=\"" << rect_top_left.x() * 1000. << "\" y=\"" << rect_top_left.y() * 1000.
        << "\" width=\"" << rect_size.x() * 1000. << "\" height=\"" << rect_size.y() * 1000.
        << "\" fill=\"" << fill_color << "\" stroke=\"" << stroke_color << "\" stroke-width=\""
        << stroke_width << "\" />";
    return oss.str();
  }

  SvgRectBuilder & top_left(double x, double y)
  {
    rect_top_left = Point(x, y);
    return *this;
  }

  SvgRectBuilder & top_left(Point p)
  {
    rect_top_left = p;
    return *this;
  }

  SvgRectBuilder & size(double x, double y)
  {
    rect_size = Point(x, y);
    return *this;
  }

  SvgRectBuilder & size(Point p)
  {
    rect_size = p;
    return *this;
  }

  SvgRectBuilder & box(const Box & box)
  {
    rect_top_left = box.min_corner();
    rect_size = box.max_corner() - box.min_corner();
    return *this;
  }

  SvgRectBuilder & fill(const std::string & color, double alpha = 1.0)
  {
    fill_color = color;
    fill_opacity = alpha;
    return *this;
  }

  SvgRectBuilder & stroke(const std::string & color, double alpha = 1.0)
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  SvgRectBuilder & strokeWidth(double width)
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgTextBuilder
{
  Point text_position;

  std::string text_string;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double font_size = 1.0;

  bool view_box_position = false;

  SvgTextBuilder() {}

  std::string getSvgString() const
  {
    std::ostringstream oss;
    oss << "<text x=\"" << text_position.x() * 1000. << "\" y=\"" << text_position.y() * 1000.
        << "\" stroke=\"" << stroke_color << "\" font-size=\"" << font_size << "\">" << text_string
        << "</text>";
    return oss.str();
  }

  SvgTextBuilder & position(double x, double y)
  {
    text_position = Point(x, y);
    return *this;
  }

  SvgTextBuilder & position(Point p)
  {
    text_position = p;
    return *this;
  }

  SvgTextBuilder & text(const std::string & text)
  {
    this->text_string = text;
    return *this;
  }

  SvgTextBuilder & stroke(const std::string & color, double alpha = 1.0)
  {
    stroke_color = color;
    ;
    stroke_opacity = alpha;
    return *this;
  }

  SvgTextBuilder & fontSize(double size)
  {
    font_size = size;
    return *this;
  }
};

struct SvgPolyLineBuilder
{
  std::vector<Point> points;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  SvgPolyLineBuilder() {}

  std::string getSvgString() const
  {
    std::ostringstream oss;
    oss << "<polyline points=\"";
    for (const auto & p : points) {
      oss << p.x() * 1000. << "," << p.y() * 1000. << " ";
    }
    oss << "\" stroke=\"" << stroke_color << "\" stroke-width=\"" << stroke_width << "\" />";
    return oss.str();
  }

  SvgPolyLineBuilder & addPoint(double x, double y)
  {
    points.emplace_back(x, y);
    return *this;
  }

  SvgPolyLineBuilder & addPoint(Point p)
  {
    points.push_back(p);
    return *this;
  }

  SvgPolyLineBuilder & strokeColor(const std::string & color, double alpha = 1.0)
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  SvgPolyLineBuilder & strokeWidth(double width)
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgPolygonBuilder
{
  std::vector<Point> points;

  std::string fill_color = "none";

  double fill_opacity = 1.;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  SvgPolygonBuilder() {}

  std::string getSvgString() const
  {
    std::ostringstream oss;
    oss << "<polygon points=\"";
    for (const auto & p : points) {
      oss << p.x() * 1000. << "," << p.y() * 1000. << " ";
    }
    oss << "\" fill=\"" << fill_color << "\" stroke=\"" << stroke_color << "\" stroke-width=\""
        << stroke_width << "\" />";
    return oss.str();
  }

  SvgPolygonBuilder & addPoint(double x, double y)
  {
    points.emplace_back(x, y);
    return *this;
  }

  SvgPolygonBuilder & addPoint(Point p)
  {
    points.push_back(p);
    return *this;
  }

  SvgPolygonBuilder & fill(const std::string & color, double alpha = 1.0)
  {
    fill_color = color;
    fill_opacity = alpha;
    return *this;
  }

  SvgPolygonBuilder & stroke(const std::string & color, double alpha = 1.0)
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  SvgPolygonBuilder & strokeWidth(double width)
  {
    stroke_width = width;
    return *this;
  }
};

struct SvgPathBuilder
{
  std::string path;

  std::string fill_color = "none";

  double fill_opacity = 1.;

  std::string stroke_color = "black";

  double stroke_opacity = 1.;

  double stroke_width = 1.0;

  SvgPathBuilder() {}

  std::string getSvgString() const
  {
    std::ostringstream oss;
    oss << "<path d=\"" << path << "\" fill=\"" << fill_color << "\" stroke=\"" << stroke_color
        << "\" stroke-width=\"" << stroke_width << "\" />";
    return oss.str();
  }

  SvgPathBuilder & pathString(const std::string & path)
  {
    this->path = path;
    return *this;
  }

  SvgPathBuilder & fillColor(const std::string & color, double alpha = 1.0)
  {
    fill_color = color;
    fill_opacity = alpha;
    return *this;
  }

  SvgPathBuilder & lineColor(const std::string & color, double alpha = 1.0)
  {
    stroke_color = color;
    stroke_opacity = alpha;
    return *this;
  }

  SvgPathBuilder & strokeWidth(double width)
  {
    stroke_width = width;
    return *this;
  }

  struct SvgPathDefinitionBuilder
  {
    std::string path;

    SvgPathDefinitionBuilder & moveTo(double x, double y)
    {
      path += "M" + std::to_string(x * 1000.) + "," + std::to_string(y * 1000.);
      return *this;
    }

    SvgPathDefinitionBuilder & moveTo(Point p) { return moveTo(p.x(), p.y()); }

    SvgPathDefinitionBuilder & lineTo(double x, double y)
    {
      path += "L" + std::to_string(x * 1000.) + "," + std::to_string(y * 1000.);
      return *this;
    }

    SvgPathDefinitionBuilder & lineTo(Point p) { return lineTo(p.x(), p.y()); }

    SvgPathDefinitionBuilder & horizontalTo(double x)
    {
      path += "H" + std::to_string(x * 1000.);
      return *this;
    }

    SvgPathDefinitionBuilder & verticalTo(double y)
    {
      path += "V" + std::to_string(y * 1000.);
      return *this;
    }

    SvgPathDefinitionBuilder & closePath()
    {
      path += "Z";
      return *this;
    }

    SvgPathDefinitionBuilder & cubicBezierTo(
      double x1, double y1, double x2, double y2, double x, double y)
    {
      path += "C" + std::to_string(x1 * 1000.) + "," + std::to_string(y1 * 1000.) + " " +
              std::to_string(x2 * 1000.) + "," + std::to_string(y2 * 1000.) + " " +
              std::to_string(x * 1000.) + "," + std::to_string(y * 1000.);
      return *this;
    }

    SvgPathDefinitionBuilder & cubicBezierTo(Point p1, Point p2, Point p)
    {
      return cubicBezierTo(p1.x(), p1.y(), p2.x(), p2.y(), p.x(), p.y());
    }

    SvgPathDefinitionBuilder & smoothCubicBezierTo(double x2, double y2, double x, double y)
    {
      path += "S" + std::to_string(x2 * 1000.) + "," + std::to_string(y2 * 1000.) + " " +
              std::to_string(x * 1000.) + "," + std::to_string(y * 1000.);
      return *this;
    }

    SvgPathDefinitionBuilder & smoothCubicBezierTo(Point p2, Point p)
    {
      return smoothCubicBezierTo(p2.x(), p2.y(), p.x(), p.y());
    }

    SvgPathDefinitionBuilder & quadraticBezierTo(double x1, double y1, double x, double y)
    {
      path += "Q" + std::to_string(x1 * 1000.) + "," + std::to_string(y1 * 1000.) + " " +
              std::to_string(x * 1000.) + "," + std::to_string(y * 1000.);
      return *this;
    }

    SvgPathDefinitionBuilder & quadraticBezierTo(Point p1, Point p)
    {
      return quadraticBezierTo(p1.x(), p1.y(), p.x(), p.y());
    }

    SvgPathDefinitionBuilder & smoothQuadraticBezierTo(double x, double y)
    {
      path += "T" + std::to_string(x * 1000.) + "," + std::to_string(y * 1000.);
      return *this;
    }

    SvgPathDefinitionBuilder & smoothQuadraticBezierTo(Point p)
    {
      return smoothQuadraticBezierTo(p.x(), p.y());
    }

    SvgPathDefinitionBuilder & arcTo(
      double rx, double ry, double x_axis_rotation, bool large_arc_flag, bool sweep_flag, double x,
      double y)
    {
      path += "A" + std::to_string(rx * 1000.) + "," + std::to_string(ry * 1000.) + " " +
              std::to_string(x_axis_rotation) + " " + std::to_string(large_arc_flag) + "," +
              std::to_string(sweep_flag) + " " + std::to_string(x * 1000.) + "," +
              std::to_string(y * 1000.);
      return *this;
    }

    SvgPathDefinitionBuilder & arcTo(
      Point r, double x_axis_rotation, bool large_arc_flag, bool sweep_flag, Point p)
    {
      return arcTo(r.x(), r.y(), x_axis_rotation, large_arc_flag, sweep_flag, p.x(), p.y());
    }

    SvgPathBuilder build()
    {
      SvgPathBuilder builder;
      builder.path = path;
      return builder;
    }
  };
};

struct CraneVisualizerBuffer
{
  using SvgPrimitiveArray = crane_visualization_interfaces::msg::SvgPrimitiveArray;
  static inline std::unique_ptr<CraneVisualizerBuffer> buffer = nullptr;

  rclcpp::Publisher<SvgPrimitiveArray>::SharedPtr publisher;

  SvgPrimitiveArray message_buffer;

  template <typename Node>
  CraneVisualizerBuffer(Node & node, const std::string topic)
  {
    publisher = node.template create_publisher<SvgPrimitiveArray>(topic, rclcpp::SensorDataQoS());
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
    if (active() && not buffer->message_buffer.primitives.empty()) {
      buffer->publisher->publish(buffer->message_buffer);
      buffer->message_buffer.primitives.clear();
    }
  }

  struct MessageBuilder
  {
    using SharedPtr = std::shared_ptr<MessageBuilder>;
    using UniquePtr = std::unique_ptr<MessageBuilder>;

    std::string layer;

    explicit MessageBuilder(const std::string & layer) : layer(layer) {}

    void flush()
    {
      if (CraneVisualizerBuffer::active()) {
        CraneVisualizerBuffer::buffer->message_buffer.layer = layer;
        CraneVisualizerBuffer::buffer->message_buffer.primitives.insert(
          CraneVisualizerBuffer::buffer->message_buffer.primitives.end(), message_buffer.begin(),
          message_buffer.end());
        message_buffer.clear();
      }
    }

    using SvgPrimitive = crane_visualization_interfaces::msg::SvgPrimitive;
    using SvgPrimitiveArray = crane_visualization_interfaces::msg::SvgPrimitiveArray;

    std::vector<SvgPrimitive> message_buffer;

    void add(const std::string & svg_string, double lifetime_s = 6000.)
    {
      SvgPrimitive primitive;
      primitive.svg_text = svg_string;
      primitive.lifetime = lifetime_s;
      message_buffer.push_back(primitive);
    }
  };
};
}  // namespace crane
#endif  // CRANE_MSG_WRAPPERS__CRANE_VISUALIZER_WRAPPER_HPP_
