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

/// 概要サーベイを実行してテキストを返す（analyze-rosbag Step 2 テンプレート移植）
std::string run_survey(const BagData & data, double sample_interval = 5.0);

}  // namespace crane::bag
