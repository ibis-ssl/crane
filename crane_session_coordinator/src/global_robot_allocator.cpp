// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_coordinator/global_robot_allocator.hpp"

#include <algorithm>
#include <crane_utils/stream.hpp>

namespace crane
{

auto GlobalRobotAllocator::allocate(
  const std::vector<SessionRequirement> & requirements,
  const std::vector<uint8_t> & available_robots, WorldModelWrapper::SharedPtr & world_model,
  const AllocationState & prev_state, const AllocationCostConfig & config)
  -> std::unordered_map<std::string, std::vector<uint8_t>>
{
  std::unordered_map<std::string, std::vector<uint8_t>> result;

  if (available_robots.empty()) {
    RCLCPP_WARN(logger_, "[GlobalRobotAllocator] 利用可能なロボットがありません");
    return result;
  }

  // ハード制約とソフト制約に分類
  std::vector<SessionRequirement> hard_requirements;
  std::vector<SessionRequirement> soft_requirements;

  for (const auto & req : requirements) {
    if (req.is_hard_constraint) {
      hard_requirements.push_back(req);
    } else {
      soft_requirements.push_back(req);
    }
  }

  // ハード制約を優先度順にソート
  std::ranges::sort(
    hard_requirements, [](const auto & a, const auto & b) { return a.priority < b.priority; });

  // ソフト制約も優先度順にソート
  std::ranges::sort(
    soft_requirements, [](const auto & a, const auto & b) { return a.priority < b.priority; });

  auto remaining_robots = available_robots;

  // Phase 1: ハード制約Sessionを先に処理
  allocateGreedy(
    hard_requirements, remaining_robots, world_model, prev_state, config, result, "ハード制約");

  // Phase 2: 残りのロボットでソフト制約Sessionをグリーディに処理
  allocateGreedy(
    soft_requirements, remaining_robots, world_model, prev_state, config, result, "ソフト制約");

  return result;
}

auto GlobalRobotAllocator::allocateGreedy(
  const std::vector<SessionRequirement> & requirements, std::vector<uint8_t> & remaining_robots,
  WorldModelWrapper::SharedPtr & world_model, const AllocationState & prev_state,
  const AllocationCostConfig & config,
  std::unordered_map<std::string, std::vector<uint8_t>> & result, const std::string & label) -> void
{
  for (const auto & req : requirements) {
    if (remaining_robots.empty()) {
      RCLCPP_WARN(
        logger_, "%sSession「%s」に割り当てるロボットが不足しています", label.c_str(),
        req.name.c_str());
      continue;
    }

    // 適性評価でロボットをスコアリング（ヒステリシスボーナスを適用して安定化）
    std::vector<std::pair<uint8_t, double>> robot_scores;
    for (const auto & robot_id : remaining_robots) {
      auto robot = world_model->getOurRobot(robot_id);
      double score = req.suitability_func(robot);
      if (prev_state.wasAssignedTo(robot_id, req.name)) {
        score -= config.hysteresis_bonus;
      }
      robot_scores.emplace_back(robot_id, score);
    }

    std::ranges::sort(robot_scores, [](const auto & a, const auto & b) {
      return a.second < b.second;  // コストが小さい順
    });

    // 必要数だけロボットを割り当て
    int num_to_allocate = std::min(req.max_robots, static_cast<int>(remaining_robots.size()));

    std::vector<uint8_t> assigned_robots;
    for (int i = 0; i < num_to_allocate; ++i) {
      assigned_robots.push_back(robot_scores[i].first);
    }

    if (static_cast<int>(assigned_robots.size()) < req.min_robots) {
      RCLCPP_WARN(
        logger_, "%sSession「%s」の最小ロボット数(%d)を満たせませんでした（実際: %lu）",
        label.c_str(), req.name.c_str(), req.min_robots, assigned_robots.size());
    }

    result[req.name] = assigned_robots;

    // 割り当てたロボットをremaining_robotsから削除
    std::erase_if(remaining_robots, [&assigned_robots](uint8_t id) {
      return std::find(assigned_robots.begin(), assigned_robots.end(), id) != assigned_robots.end();
    });

    RCLCPP_DEBUG_STREAM(
      logger_, label << "Session「" << req.name << "」に" << assigned_robots.size()
                     << "ロボットを割り当て: " << assigned_robots);
  }
}

}  // namespace crane
