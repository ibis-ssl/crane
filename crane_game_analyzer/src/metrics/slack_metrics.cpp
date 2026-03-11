// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_game_analyzer/metrics/slack_metrics.hpp"

#include <string>

namespace
{
void computeSlackForTeam(
  const std::vector<std::shared_ptr<crane::RobotInfo>> & robots,
  crane::WorldModelWrapper * world_model, std::vector<crane_msgs::msg::Slack> & output)
{
  for (const auto & robot : robots) {
    crane::RobotList single_robot{robot};
    auto [min_slack, max_slack] =
      world_model->getMinMaxSlackInterceptPointAndSlackTime(single_robot);

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

    output.push_back(slack_msg);
  }
}
}  // namespace

namespace crane::metrics
{

// OurSlackMetric実装

OurSlackMetric::OurSlackMetric() : MetricBase(MetricId::OUR_SLACK, "OurSlack") {}

auto OurSlackMetric::compute(MetricContext & ctx) -> void
{
  computeSlackForTeam(
    ctx.world_model->ours().robotsWhere().available().get(), ctx.world_model,
    ctx.analysis.our_slack);
}

auto OurSlackMetric::visualize(
  MetricContext & ctx, const VisualizerMessageBuilder::SharedPtr & visualizer) -> void
{
  if (!visualizer) {
    return;
  }

  for (const auto & robot : ctx.world_model->ours().robotsWhere().available().get()) {
    const crane_msgs::msg::Slack * slack_data = nullptr;
    for (const auto & s : ctx.analysis.our_slack) {
      if (s.id == robot->id) {
        slack_data = &s;
        break;
      }
    }
    if (!slack_data) continue;

    if (slack_data->min.slack_time != 0.0) {
      visualizer->text()
        .position(robot->pose.pos.x(), robot->pose.pos.y() - 0.3)
        .text("min slack: " + std::to_string(slack_data->min.slack_time))
        .fill("white")
        .fontSize(100)
        .build();
      visualizer->line()
        .start(robot->pose.pos)
        .end(Point(slack_data->min.x, slack_data->min.y))
        .stroke("red", 0.5)
        .strokeWidth(5)
        .build();
    }

    if (slack_data->max.slack_time > 0.0) {
      visualizer->text()
        .position(robot->pose.pos.x(), robot->pose.pos.y() - 0.5)
        .text("max slack: " + std::to_string(slack_data->max.slack_time))
        .fill("white")
        .fontSize(100)
        .build();
      visualizer->line()
        .start(robot->pose.pos)
        .end(Point(slack_data->max.x, slack_data->max.y))
        .stroke("red", 0.5)
        .strokeWidth(5)
        .build();
    }
  }
}

// TheirSlackMetric実装

TheirSlackMetric::TheirSlackMetric() : MetricBase(MetricId::THEIR_SLACK, "TheirSlack") {}

auto TheirSlackMetric::compute(MetricContext & ctx) -> void
{
  computeSlackForTeam(
    ctx.world_model->theirs().robotsWhere().available().get(), ctx.world_model,
    ctx.analysis.their_slack);
}

}  // namespace crane::metrics
