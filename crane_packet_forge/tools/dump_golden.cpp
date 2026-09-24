// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// test/golden_vectors.json を再生成する。中身の正本は C++ 側 (robot_packet.h)。
//
//   colcon build --packages-select crane_packet_forge
//   ros2 run crane_packet_forge dump_golden > test/golden_vectors.json
//
// 生成した JSON は C++ の test_golden_vectors と Python の test_assemble_golden が
// 同じ基準として読む。両方が同じバイト列を作れなくなったら、どちらかが落ちる。

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../test/golden_vectors.hpp"

namespace
{

auto formatNumber(double value) -> std::string
{
  char buffer[64];
  if (value == static_cast<int64_t>(value)) {
    std::snprintf(buffer, sizeof(buffer), "%" PRId64, static_cast<int64_t>(value));
  } else {
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
  }
  return buffer;
}

}  // namespace

auto main() -> int
{
  const auto vectors = crane_packet_forge::goldenVectors();

  // convertFloatToTwoByte はレンジ外の値で std::cout に警告を出す。
  // clamp 系のベクタでは必ず出るので、先に cout を退避して hex だけ集める。
  // そうしないと警告が JSON に混ざって壊れる。
  std::vector<std::string> hexes;
  {
    std::ostringstream sink;
    auto * saved = std::cout.rdbuf(sink.rdbuf());
    for (const auto & vector : vectors) {
      hexes.push_back(
        crane_packet_forge::serializeToHex(crane_packet_forge::makeCommand(vector.fields)));
    }
    std::cout.rdbuf(saved);
  }

  std::cout << "{\n";
  std::cout << "  \"_comment\": \"robot_packet.h から生成した基準ベクタ。"
               "再生成は ros2 run crane_packet_forge dump_golden。手で編集しない\",\n";
  std::cout << "  \"base\": \"neutral\",\n";
  std::cout << "  \"vectors\": [\n";

  for (size_t i = 0; i < vectors.size(); ++i) {
    const auto & vector = vectors[i];
    std::cout << "    {\n";
    std::cout << "      \"name\": \"" << vector.name << "\",\n";
    std::cout << "      \"fields\": {";
    for (size_t j = 0; j < vector.fields.size(); ++j) {
      std::cout << (j == 0 ? "\n" : ",\n");
      std::cout << "        \"" << vector.fields[j].key
                << "\": " << formatNumber(vector.fields[j].value);
    }
    std::cout << (vector.fields.empty() ? "}" : "\n      }") << ",\n";
    std::cout << "      \"hex\": \"" << hexes[i] << "\"\n";
    std::cout << "    }" << (i + 1 == vectors.size() ? "\n" : ",\n");
  }

  std::cout << "  ]\n";
  std::cout << "}\n";
  return 0;
}
