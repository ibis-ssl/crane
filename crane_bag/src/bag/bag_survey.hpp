// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <string>

#include "bag_reader.hpp"

namespace crane::bag
{

/// survey が使うサンプリング間隔[秒]。read() の読み込み時ダウンサンプルと
/// 各セクションの sample() で同一値を共有し、出力を変えずに展開件数を減らす。
constexpr double kSurveySampleInterval = 5.0;     ///< world_model / game_analysis セクション
constexpr double kSurveyVelocityInterval = 10.0;  ///< robot_commands 速度セクション

/// 概要サーベイを実行してテキストを返す（analyze-rosbag Step 2 テンプレート移植）
std::string run_survey(const BagData & data, double sample_interval = kSurveySampleInterval);

}  // namespace crane::bag
