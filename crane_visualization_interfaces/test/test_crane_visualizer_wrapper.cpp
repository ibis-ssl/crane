// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "crane_visualization_interfaces/crane_visualizer_wrapper.hpp"

namespace crane
{
namespace
{
// rect() と各描画関数は shared_from_this() を使うので shared_ptr で作る
auto makeBuilder() { return std::make_shared<VisualizerMessageBuilder>("test"); }
}  // namespace

// SVG の rect は y から下（SVG y が増える＝フィールド y が減る向き）へ伸びるので、
// 左上はフィールドの (min.x, max.y) になる
TEST(SvgRectBuilder, BoxUsesFieldTopLeftCorner)
{
  auto builder = makeBuilder();
  builder->rect().box(Box{Point(1.0, 2.0), Point(3.0, 5.0)}).stroke("yellow").build();

  ASSERT_EQ(builder->message_buffer.size(), 1u);
  EXPECT_EQ(
    builder->message_buffer[0],
    "<rect x=\"1000.000\" y=\"-5000.000\" width=\"2000.000\" height=\"3000.000\" "
    "fill=\"none\" fill-opacity=\"1.00\" stroke=\"yellow\" "
    "stroke-opacity=\"1.00\" stroke-width=\"1.00\" />");
}

TEST(VisualizerMessageBuilder, DrawFieldRectDrawsFourEdgesFromTopLeftClockwise)
{
  auto builder = makeBuilder();
  builder->drawFieldRect(Point(3.0, 2.0), Point(1.0, 5.0), "white", 10.0);

  auto line = [](
                const std::string & x1, const std::string & y1, const std::string & x2,
                const std::string & y2) {
    return "<line x1=\"" + x1 + "\" y1=\"" + y1 + "\" x2=\"" + x2 + "\" y2=\"" + y2 +
           "\" stroke=\"white\" stroke-opacity=\"1.00\" stroke-width=\"10.00\" />";
  };
  const std::vector<std::string> expected{
    line("1000.000", "-5000.000", "3000.000", "-5000.000"),
    line("3000.000", "-5000.000", "3000.000", "-2000.000"),
    line("3000.000", "-2000.000", "1000.000", "-2000.000"),
    line("1000.000", "-2000.000", "1000.000", "-5000.000"),
  };
  EXPECT_EQ(builder->message_buffer, expected);
}
}  // namespace crane
