// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_msg_wrappers/crane_visualizer_wrapper.hpp"

namespace crane
{
void VisualizerMessageBuilder::flush()
{
  if (CraneVisualizerBuffer::active()) {
    SvgPrimitiveArray layer_msg;
    layer_msg.layer = layer;
    layer_msg.svg_primitives = message_buffer;
    CraneVisualizerBuffer::buffer->message_buffer.svg_primitive_arrays.push_back(layer_msg);
    message_buffer.clear();
  }
}

void VisualizerMessageBuilder::clearBuffer()
{
  clear();
  if (CraneVisualizerBuffer::active()) {
    CraneVisualizerBuffer::clear(layer);
  }
}

auto VisualizerMessageBuilder::circle() -> SvgCircleBuilder
{
  return SvgCircleBuilder(shared_from_this());
}

auto VisualizerMessageBuilder::line() -> SvgLineBuilder
{
  return SvgLineBuilder(shared_from_this());
}

auto VisualizerMessageBuilder::polygon() -> SvgPolygonBuilder
{
  return SvgPolygonBuilder(shared_from_this());
}

auto VisualizerMessageBuilder::polyline() -> SvgPolyLineBuilder
{
  return SvgPolyLineBuilder(shared_from_this());
}

auto VisualizerMessageBuilder::text() -> SvgTextBuilder
{
  return SvgTextBuilder(shared_from_this());
}

auto VisualizerMessageBuilder::rect() -> SvgRectBuilder
{
  return SvgRectBuilder(shared_from_this());
}

}  // namespace crane
