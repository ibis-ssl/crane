// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_controller/global_robot_allocator.hpp"

#include <algorithm>
#include <crane_utils/stream.hpp>
#include <limits>
#include <sstream>

namespace crane
{

auto GlobalRobotAllocator::allocate(
  const std::vector<TacticRequirement> & requirements,
  const std::vector<uint8_t> & available_robots, WorldModelWrapper::SharedPtr & world_model,
  const AllocationState & prev_state, const AllocationCostConfig & config)
  -> std::unordered_map<std::string, std::vector<uint8_t>>
{
  std::unordered_map<std::string, std::vector<uint8_t>> result;

  if (available_robots.empty()) {
    RCLCPP_DEBUG(logger_, "GlobalRobotAllocator: 利用可能なロボットがありません");
    return result;
  }

  // ハード制約とソフト制約に分類
  std::vector<TacticRequirement> hard_requirements;
  std::vector<TacticRequirement> soft_requirements;

  for (const auto & req : requirements) {
    if (req.is_hard_constraint) {
      hard_requirements.push_back(req);
    } else {
      soft_requirements.push_back(req);
    }
  }

  // ハード制約を優先度順にソート
  std::ranges::sort(hard_requirements, [](const auto & a, const auto & b) {
    return a.priority < b.priority;
  });

  // ソフト制約も優先度順にソート
  std::ranges::sort(soft_requirements, [](const auto & a, const auto & b) {
    return a.priority < b.priority;
  });

  auto remaining_robots = available_robots;

  // Phase 1: ハード制約Tacticを先に処理
  allocateHardConstraints(
    hard_requirements, remaining_robots, world_model, prev_state, config, result);

  // Phase 2: 残りのロボットでソフト制約Tacticをハンガリアン法で処理
  allocateSoftConstraints(
    soft_requirements, remaining_robots, world_model, prev_state, config, result);

  return result;
}

auto GlobalRobotAllocator::allocateHardConstraints(
  const std::vector<TacticRequirement> & hard_requirements,
  std::vector<uint8_t> & remaining_robots, WorldModelWrapper::SharedPtr & world_model,
  const AllocationState & prev_state, const AllocationCostConfig & config,
  std::unordered_map<std::string, std::vector<uint8_t>> & result) -> void
{
  for (const auto & req : hard_requirements) {
    if (remaining_robots.empty()) {
      RCLCPP_WARN(
        logger_, "ハード制約Tactic「%s」に割り当てるロボットが不足しています", req.name.c_str());
      break;
    }

    // 適性評価でロボットをソート
    std::vector<std::pair<uint8_t, double>> robot_scores;
    for (const auto & robot_id : remaining_robots) {
      auto robot = world_model->getOurRobot(robot_id);
      double score = req.suitability_func(robot);
      robot_scores.emplace_back(robot_id, score);
    }

    std::ranges::sort(robot_scores, [](const auto & a, const auto & b) {
      return a.second < b.second;  // コストが小さい順
    });

    // 必要数だけロボットを割り当て
    int num_to_allocate = std::min(req.max_robots, static_cast<int>(remaining_robots.size()));
    num_to_allocate = std::max(req.min_robots, num_to_allocate);

    std::vector<uint8_t> assigned_robots;
    for (int i = 0; i < num_to_allocate && i < robot_scores.size(); ++i) {
      assigned_robots.push_back(robot_scores[i].first);
    }

    result[req.name] = assigned_robots;

    // 割り当てたロボットをremaining_robotsから削除
    for (const auto & robot_id : assigned_robots) {
      auto it = std::find(remaining_robots.begin(), remaining_robots.end(), robot_id);
      if (it != remaining_robots.end()) {
        remaining_robots.erase(it);
      }
    }

    RCLCPP_DEBUG(
      logger_, "ハード制約Tactic「%s」に%luロボットを割り当て: %s", req.name.c_str(),
      assigned_robots.size(), [&]() {
        std::stringstream ss;
        for (const auto & id : assigned_robots) {
          ss << static_cast<int>(id) << " ";
        }
        return ss.str();
      }()
                     .c_str());
  }
}

auto GlobalRobotAllocator::allocateSoftConstraints(
  const std::vector<TacticRequirement> & soft_requirements,
  const std::vector<uint8_t> & remaining_robots, WorldModelWrapper::SharedPtr & world_model,
  const AllocationState & prev_state, const AllocationCostConfig & config,
  std::unordered_map<std::string, std::vector<uint8_t>> & result) -> void
{
  if (remaining_robots.empty() || soft_requirements.empty()) {
    return;
  }

  // 仮想ターゲットを生成（各Tacticが必要とするロボット数分）
  std::vector<VirtualTarget> virtual_targets;
  for (const auto & req : soft_requirements) {
    int num_targets = std::min(req.max_robots, static_cast<int>(remaining_robots.size()));
    for (int i = 0; i < num_targets; ++i) {
      virtual_targets.push_back(
        {req.name, req.priority, static_cast<size_t>(i), req.suitability_func});
    }
  }

  if (virtual_targets.empty()) {
    return;
  }

  // ロボット数 ≤ ターゲット数 を保証
  if (remaining_robots.size() > virtual_targets.size()) {
    RCLCPP_WARN(
      logger_,
      "GlobalRobotAllocator: ロボット数(%lu)がターゲット数(%lu)を超えています。一部のロボットは割り当てられません",
      remaining_robots.size(), virtual_targets.size());
    // ハンガリアン法は robots <= targets を要求するため、
    // ダミーターゲットを追加する必要があるが、ここでは単純に警告を出す
  }

  // コスト関数を構築
  auto cost_func = buildCostMatrix(remaining_robots, virtual_targets, world_model, prev_state, config);

  // ハンガリアン法で最適割当を計算
  auto assignment = getOptimalAssignmentsWithCost(
    remaining_robots.size(), virtual_targets.size(), cost_func);

  // 割当結果を集計
  std::unordered_map<std::string, std::vector<uint8_t>> tactic_assignments;
  for (size_t robot_idx = 0; robot_idx < assignment.size(); ++robot_idx) {
    int target_idx = assignment[robot_idx];
    if (target_idx < 0 || target_idx >= virtual_targets.size()) {
      continue;
    }

    const auto & target = virtual_targets[target_idx];
    uint8_t robot_id = remaining_robots[robot_idx];
    tactic_assignments[target.tactic_name].push_back(robot_id);
  }

  // min_robotsの制約を満たしているか確認
  for (const auto & req : soft_requirements) {
    auto it = tactic_assignments.find(req.name);
    if (it != tactic_assignments.end()) {
      if (it->second.size() < req.min_robots) {
        RCLCPP_WARN(
          logger_, "Tactic「%s」の最小ロボット数(%d)を満たせませんでした（実際: %lu）",
          req.name.c_str(), req.min_robots, it->second.size());
      }
    } else if (req.min_robots > 0) {
      RCLCPP_WARN(
        logger_, "Tactic「%s」にロボットを割り当てられませんでした（最小要求: %d）",
        req.name.c_str(), req.min_robots);
    }
  }

  // 結果をマージ
  for (const auto & [tactic_name, robots] : tactic_assignments) {
    result[tactic_name] = robots;
    RCLCPP_DEBUG(
      logger_, "ソフト制約Tactic「%s」に%luロボットを割り当て: %s", tactic_name.c_str(),
      robots.size(), [&]() {
        std::stringstream ss;
        for (const auto & id : robots) {
          ss << static_cast<int>(id) << " ";
        }
        return ss.str();
      }()
                     .c_str());
  }
}

auto GlobalRobotAllocator::buildCostMatrix(
  const std::vector<uint8_t> & robots, const std::vector<VirtualTarget> & targets,
  WorldModelWrapper::SharedPtr & world_model, const AllocationState & prev_state,
  const AllocationCostConfig & config) -> std::function<double(size_t, size_t)>
{
  return [&robots, &targets, &world_model, &prev_state, &config](
           size_t robot_idx, size_t target_idx) -> double {
    uint8_t robot_id = robots[robot_idx];
    const auto & target = targets[target_idx];
    auto robot = world_model->getOurRobot(robot_id);

    // 基本コスト: ロボットとTacticの適性（ダミーターゲット位置を使用しないため、スコアのみ）
    // ここでは、各Tacticの suitability_func を直接呼び出すことはできないため、
    // 簡易的に優先度コストのみを使用
    double base_cost = 0.0;

    // ヒステリシスコンテキストを構築
    AssignmentContext context;
    context.was_assigned_to_same_tactic = prev_state.wasAssignedTo(robot_id, target.tactic_name);
    context.tactic_priority = target.tactic_priority;

    // 簡易コスト計算（位置ベースではなく優先度ベース）
    double priority_cost = context.tactic_priority * config.priority_cost_multiplier;
    double hysteresis_bonus = context.was_assigned_to_same_tactic ? config.hysteresis_bonus : 0.0;
    double velocity_hysteresis = 0.0;
    if (context.was_assigned_to_same_tactic) {
      velocity_hysteresis = robot->vel.linear.norm() * config.velocity_hysteresis_factor;
    }

    return base_cost + priority_cost - hysteresis_bonus + velocity_hysteresis;
  };
}

}  // namespace crane
