// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// robot_packet.h のシリアライザが golden_vectors.json を再現することを確かめる。
// Python 側 (test_assemble_golden.py) が同じ JSON と突き合わせるので、
// C++ と Python のどちらがずれても、どちらかのテストが落ちる。
//
// JSON は自前で生成したものだけを読むので、"name" と "hex" を拾う最小の走査で足りる。
// そのためにライブラリ依存を増やさない。

#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include "golden_vectors.hpp"

namespace
{

/// `"key": "value"` の value を order 通りに拾う。
auto extractStrings(const std::string & text, const std::string & key) -> std::vector<std::string>
{
  const std::string needle = "\"" + key + "\": \"";
  std::vector<std::string> out;
  size_t position = 0;
  while ((position = text.find(needle, position)) != std::string::npos) {
    const size_t start = position + needle.size();
    const size_t end = text.find('"', start);
    if (end == std::string::npos) {
      break;
    }
    out.push_back(text.substr(start, end - start));
    position = end;
  }
  return out;
}

auto loadGolden() -> std::map<std::string, std::string>
{
  std::ifstream file(GOLDEN_VECTORS_PATH);
  EXPECT_TRUE(file.is_open()) << GOLDEN_VECTORS_PATH << " を開けない";
  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::string text = buffer.str();

  const auto names = extractStrings(text, "name");
  const auto hexes = extractStrings(text, "hex");
  EXPECT_EQ(names.size(), hexes.size());

  std::map<std::string, std::string> out;
  for (size_t i = 0; i < names.size() && i < hexes.size(); ++i) {
    out[names[i]] = hexes[i];
  }
  return out;
}

}  // namespace

TEST(GoldenVectors, SerializerReproducesCommittedBytes)
{
  const auto golden = loadGolden();
  const auto vectors = crane_packet_forge::goldenVectors();
  ASSERT_FALSE(golden.empty()) << "golden_vectors.json が空";
  ASSERT_EQ(golden.size(), vectors.size())
    << "golden_vectors.json とベクタ定義の数が合わない。dump_golden で再生成する";

  for (const auto & vector : vectors) {
    const auto found = golden.find(vector.name);
    ASSERT_NE(found, golden.end()) << vector.name << " が golden_vectors.json に無い";
    const auto actual =
      crane_packet_forge::serializeToHex(crane_packet_forge::makeCommand(vector.fields));
    EXPECT_EQ(actual, found->second)
      << vector.name << " のバイト列が基準と違う。robot_packet.h を変えたなら "
      << "ros2 run crane_packet_forge dump_golden > test/golden_vectors.json で再生成し、"
      << "Orion_CM4 と G474 の robot_packet.h も同期する";
  }
}

TEST(GoldenVectors, EveryFieldKeyIsRecognized)
{
  // 綴りを間違えたフィールドが黙って無視されると、ベクタが薄くなったことに気づけない。
  for (const auto & vector : crane_packet_forge::goldenVectors()) {
    RobotCommandV2 command = crane_packet_forge::neutralCommand();
    for (const auto & field : vector.fields) {
      EXPECT_TRUE(crane_packet_forge::applyField(&command, field))
        << vector.name << " の " << field.key << " は applyField が知らないキー";
    }
  }
}

TEST(GoldenVectors, UnusedBytesAreZero)
{
  // serialize は union の非選択側 (28-31) と 38-63 に触らない。
  // 基準はゼロ初期化した状態なので、そこが 0 であることを固定しておく。
  for (const auto & vector : crane_packet_forge::goldenVectors()) {
    const auto hex =
      crane_packet_forge::serializeToHex(crane_packet_forge::makeCommand(vector.fields));
    for (int index = 28; index < 64; ++index) {
      if (index >= 32 && index <= 37) {
        continue;  // target_global_pos と terminal_velocity は書かれる
      }
      EXPECT_EQ(hex.substr(index * 2, 2), "00")
        << vector.name << " の byte " << index << " が 0 でない";
    }
  }
}
