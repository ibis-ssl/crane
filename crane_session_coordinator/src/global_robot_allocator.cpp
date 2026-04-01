// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_coordinator/global_robot_allocator.hpp"

#include <algorithm>
#include <crane_utils/stream.hpp>
#include <unordered_set>

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

  // 全requirementsを優先度順にソートして1回で割当
  auto sorted_requirements = requirements;
  std::ranges::sort(
    sorted_requirements, [](const auto & a, const auto & b) { return a.priority < b.priority; });

  auto remaining_robots = available_robots;

  for (const auto & req : sorted_requirements) {
    if (remaining_robots.empty()) {
      break;
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

    result[req.name] = assigned_robots;

    // 割り当てたロボットをremaining_robotsから削除
    const std::unordered_set<uint8_t> assigned_set(assigned_robots.begin(), assigned_robots.end());
    std::erase_if(
      remaining_robots, [&assigned_set](uint8_t id) { return assigned_set.count(id) > 0; });

    RCLCPP_DEBUG_STREAM(
      logger_, "Session「" << req.name << "」に" << assigned_robots.size()
                           << "ロボットを割り当て: " << assigned_robots);
  }

  if (!remaining_robots.empty()) {
    RCLCPP_WARN(
      logger_,
      "[GlobalRobotAllocator] %zu台のロボットがどのセッションにも割り当てられませんでした: %s",
      remaining_robots.size(),
      [&]() {
        std::string ids;
        for (auto id : remaining_robots) {
          ids += std::to_string(id) + " ";
        }
        return ids;
      }()
        .c_str());
  }

  return result;
}

}  // namespace crane
