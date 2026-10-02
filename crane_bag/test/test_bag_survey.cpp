// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "bag_survey.hpp"

namespace cb = crane::bag;

TEST(RunSurvey, WorldModelSectionShowsPlanarBallSpeed)
{
  cb::BagData data;
  data.info.start_time_ns = 0;
  const double vx[] = {2.0, 2.2};
  const double vy[] = {2.0, -2.2};
  for (int i = 0; i < 2; ++i) {
    cb::TimestampedMsg<cb::WorldModel> f;
    f.timestamp_ns = static_cast<int64_t>(i + 1) * 5'000'000'000;
    f.msg.ball_info.position = {1.0, 2.0, 0.0};
    f.msg.ball_info.velocity = {vx[i], vy[i]};
    data.world_models.push_back(f);
  }

  const std::string survey = cb::run_survey(data, 5.0);
  EXPECT_NE(survey.find("  t=5.00: ball=(1.000,2.000) speed=2.83m/s\n"), std::string::npos)
    << survey;
  EXPECT_NE(survey.find("  t=10.00: ball=(1.000,2.000) speed=3.11m/s\n"), std::string::npos)
    << survey;
}
