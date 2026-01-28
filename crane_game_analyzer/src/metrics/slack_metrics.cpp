// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/slack_metrics.hpp"

namespace crane::metrics
{

// OurSlackMetric実装

OurSlackMetric::OurSlackMetric() : MetricBase(MetricId::OUR_SLACK, "OurSlack") {}

auto OurSlackMetric::compute(MetricContext & ctx) -> void
{
  for (const auto & robot : ctx.world_model->ours().robotsWhere().available().get()) {
    RobotList single_robot{robot};
    auto [min_slack, max_slack] =
      ctx.world_model->getMinMaxSlackInterceptPointAndSlackTime(single_robot);

    crane_msgs::msg::Slack slack_msg;
    slack_msg.id = robot->id;

    if (min_slack) {
      slack_msg.min.slack_time = min_slack->slack_time;
      slack_msg.min.x = min_slack->intercept_point.x();
      slack_msg.min.y = min_slack->intercept_point.y();
    }

    if (max_slack) {
      slack_msg.max.slack_time = max_slack->slack_time;
      slack_msg.max.x = max_slack->intercept_point.x();
      slack_msg.max.y = max_slack->intercept_point.y();
    }

    ctx.analysis.our_slack.push_back(slack_msg);
  }
}

// TheirSlackMetric実装

TheirSlackMetric::TheirSlackMetric() : MetricBase(MetricId::THEIR_SLACK, "TheirSlack") {}

auto TheirSlackMetric::compute(MetricContext & ctx) -> void
{
  for (const auto & robot : ctx.world_model->theirs().robotsWhere().available().get()) {
    RobotList single_robot{robot};
    auto [min_slack, max_slack] =
      ctx.world_model->getMinMaxSlackInterceptPointAndSlackTime(single_robot);

    crane_msgs::msg::Slack slack_msg;
    slack_msg.id = robot->id;

    if (min_slack) {
      slack_msg.min.slack_time = min_slack->slack_time;
      slack_msg.min.x = min_slack->intercept_point.x();
      slack_msg.min.y = min_slack->intercept_point.y();
    }

    if (max_slack) {
      slack_msg.max.slack_time = max_slack->slack_time;
      slack_msg.max.x = max_slack->intercept_point.x();
      slack_msg.max.y = max_slack->intercept_point.y();
    }

    ctx.analysis.their_slack.push_back(slack_msg);
  }
}

}  // namespace crane::metrics
