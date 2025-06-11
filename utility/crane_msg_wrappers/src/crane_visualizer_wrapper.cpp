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
    SvgPrimitiveArray layer_msg;
    layer_msg.layer = layer;
    layer_msg.svg_primitives = message_buffer;
    CraneVisualizerBuffer::buffer->message_buffer.svg_primitive_arrays.push_back(layer_msg);
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
}  // namespace crane
