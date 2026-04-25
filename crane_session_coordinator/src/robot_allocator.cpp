// Copyright (c) 2022 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "crane_session_coordinator/robot_allocator.hpp"

#include <algorithm>
#include <crane_utils/stream.hpp>
#include <range/v3/action/sort.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/transform.hpp>
#include <sstream>
#include <unordered_set>

namespace crane
{

RobotAllocator::RobotAllocator(
  std::shared_ptr<ConfigurationManager> config_manager,
  std::shared_ptr<SessionRegistry> tactic_registry, rclcpp::Logger logger)
: config_manager_(config_manager), session_registry_(tactic_registry), logger_(logger)
{
}

auto RobotAllocator::allocate(
  const std::string & session_name, std::vector<uint8_t> selectable_robot_ids,
  WorldModelWrapper::SharedPtr & world_model, rclcpp::Node & node)
  -> crane_msgs::msg::RobotSelectResults
{
  auto session_capacities_opt = config_manager_->getSessionCapacitiesForSituation(session_name);
  if (!session_capacities_opt.has_value()) {
    RCLCPP_ERROR(
      logger_,
      "\t「%"
      "s」というSituationに対してロボット割当リクエストが発行されましたが，見つかりませんでした",
      session_name.c_str());
    return crane_msgs::msg::RobotSelectResults{};
  }

  const auto & session_capacities = session_capacities_opt.value();

  // 前回のプランナーリストを保存し、新しいリストをクリア
  auto prev_available_planners = session_registry_->getAllPlanners();
  session_registry_->clear();

  // SessionRequirementリストを構築
  std::vector<SessionRequirement> requirements;
  int priority = 0;
  for (const auto & session_capacity : session_capacities) {
    if (session_capacity.max_robots <= 0) {
      continue;
    }

    // プランナー生成
    auto session = session_registry_->getOrCreatePlanner(
      session_capacity.session_name, world_model, node, prev_available_planners,
      session_capacity.params);

    // 適性関数を取得
    auto suitability_func = session->getRobotSuitabilityFunc();

    // 動的ロボット数を取得してクランプ
    int desired = session->getDesiredRobotNumber(session_capacity.max_robots);
    int effective_max = std::min(desired, session_capacity.max_robots);

    requirements.emplace_back(
      session_capacity.session_name, priority++,
      effective_max,  // max_robots（動的に調整）
      suitability_func, session_capacity.fixed_robots);
  }

  // グリーディ方式でロボットを割当
  auto allocation = allocateRobotsGreedy(
    requirements, selectable_robot_ids, world_model, allocation_state_, allocation_cost_config_);

  // 割当結果を適用
  crane_msgs::msg::RobotSelectResults results;
  for (const auto & [allocated_name, robot_ids] : allocation) {
    // session_capacitiesを1回だけ検索（フォールバック生成とmin/max取得の両方で使い回す）
    auto capacity_it = std::find_if(
      session_capacities.begin(), session_capacities.end(),
      [&allocated_name](const auto & s) { return s.session_name == allocated_name; });

    // Sessionを取得または再生成
    auto session_it = std::find_if(
      session_registry_->getAllPlanners().begin(), session_registry_->getAllPlanners().end(),
      [&allocated_name](const auto & t) { return t->name == allocated_name; });

    SessionBase::SharedPtr session;
    if (session_it != session_registry_->getAllPlanners().end()) {
      session = *session_it;
    } else if (capacity_it != session_capacities.end()) {
      // 見つからない場合は新規生成（通常はここには来ない）
      session = session_registry_->getOrCreatePlanner(
        allocated_name, world_model, node, prev_available_planners, capacity_it->params);
    }

    if (session) {
      // ロボット割当をSessionに反映
      // GlobalRobotAllocatorが選択したロボットを直接設定（getSelectedRobotsをバイパス）
      session->setAllocatedRobots(robot_ids);

      // レジストリに追加
      if (session_it == session_registry_->getAllPlanners().end()) {
        session_registry_->addPlanner(session);
      }

      // AllocationStateを更新（ターゲット位置は現時点では不明なのでロボット位置を使用）
      for (auto id : robot_ids) {
        auto robot = world_model->getOurRobot(id);
        allocation_state_.updateAssignment(id, allocated_name, robot->pose.pos);
        prev_robot_roles_.insert_or_assign(id, RobotRole{allocated_name, ""});
      }

      // RobotSelectResult を構築
      crane_msgs::msg::RobotSelectResult result;
      result.name = allocated_name;

      if (capacity_it != session_capacities.end()) {
        result.max_robots_num = static_cast<uint8_t>(capacity_it->max_robots);
      }

      result.selectable_robots_num = static_cast<uint8_t>(selectable_robot_ids.size());
      result.selectable_robots = selectable_robot_ids;
      result.selected_robots = robot_ids;
      results.results.push_back(result);
    }
  }

  return results;
}

auto RobotAllocator::detectRobotChange(const std::vector<uint8_t> & observed_robot_ids) const
  -> bool
{
  auto assigned_robot_ids = getAssignedRobotIds();
  auto sorted_observed_ids = observed_robot_ids;
  std::sort(sorted_observed_ids.begin(), sorted_observed_ids.end());

  if (assigned_robot_ids.size() != sorted_observed_ids.size()) {
    RCLCPP_DEBUG_STREAM(
      logger_, "ロボットの数が変動しています｜割当数：" << assigned_robot_ids.size() << ", 観測数："
                                                        << sorted_observed_ids.size());
    return true;
  } else if (assigned_robot_ids != sorted_observed_ids) {
    RCLCPP_DEBUG_STREAM(
      logger_, "ロボットの数は変わっていないですが、ラインナップが変動しています\n"
                 << "\tbefore: " << assigned_robot_ids << "\tafter : " << sorted_observed_ids);
    return true;
  }

  return false;
}

auto RobotAllocator::getAssignedRobotIds() const -> std::vector<uint8_t>
{
  auto assigned_robot_ids =
    session_registry_->getAllPlanners() |
    ranges::views::transform([](const auto & session) { return session->getRobots(); }) |
    ranges::views::join | ranges::views::transform([](const auto & robot) { return robot.id; }) |
    ranges::to<std::vector>() | ranges::actions::sort;
  return assigned_robot_ids;
}

auto RobotAllocator::buildAssignmentLog() const -> std::string
{
  std::stringstream assignment_log;
  bool first = true;
  for (const auto & session : session_registry_->getAllPlanners()) {
    if (!first) {
      assignment_log << ", ";
    }
    first = false;
    assignment_log << session->name << ":[";
    const auto & robots = session->getRobots();
    for (size_t i = 0; i < robots.size(); ++i) {
      if (i > 0) {
        assignment_log << ",";
      }
      assignment_log << static_cast<int>(robots[i].id);
    }
    assignment_log << "]";
  }
  return assignment_log.str();
}

auto RobotAllocator::logAssignmentIfChanged(const std::string & current_assignment) -> void
{
  if (current_assignment != prev_assignment_log_) {
    if (current_assignment.empty()) {
      RCLCPP_DEBUG(logger_, "ロボット割当: なし");
    } else {
      RCLCPP_DEBUG(logger_, "ロボット割当: %s", current_assignment.c_str());
    }
    prev_assignment_log_ = current_assignment;
  }
}

auto RobotAllocator::allocateRobotsGreedy(
  const std::vector<SessionRequirement> & requirements,
  const std::vector<uint8_t> & available_robots, WorldModelWrapper::SharedPtr & world_model,
  const AllocationState & prev_state, const AllocationCostConfig & config)
  -> std::unordered_map<std::string, std::vector<uint8_t>>
{
  std::unordered_map<std::string, std::vector<uint8_t>> result;

  if (available_robots.empty()) {
    RCLCPP_WARN(logger_, "利用可能なロボットがありません");
    return result;
  }

  auto sorted_requirements = requirements;
  std::ranges::sort(
    sorted_requirements, [](const auto & a, const auto & b) { return a.priority < b.priority; });

  auto remaining_robots = available_robots;

  for (const auto & req : sorted_requirements) {
    if (remaining_robots.empty()) {
      RCLCPP_WARN(logger_, "Session「%s」に割り当てるロボットが不足しています", req.name.c_str());
      break;
    }

    std::vector<uint8_t> assigned_robots;

    // fixed_robots 指定時は suitability を無視して指定IDを優先取得
    if (!req.fixed_robots.empty()) {
      const std::unordered_set<uint8_t> remaining_set(
        remaining_robots.begin(), remaining_robots.end());
      const int num_to_allocate = std::min(req.max_robots, static_cast<int>(req.fixed_robots.size()));
      for (int i = 0; i < num_to_allocate; ++i) {
        const uint8_t id = req.fixed_robots[i];
        if (remaining_set.count(id) > 0) {
          assigned_robots.push_back(id);
        } else {
          RCLCPP_WARN(
            logger_, "Session「%s」の固定割当ID %d は利用不可のためスキップ", req.name.c_str(),
            static_cast<int>(id));
        }
      }
    } else {
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

      std::ranges::sort(
        robot_scores, [](const auto & a, const auto & b) { return a.second < b.second; });

      const int num_to_allocate =
        std::min(req.max_robots, static_cast<int>(remaining_robots.size()));
      for (int i = 0; i < num_to_allocate; ++i) {
        assigned_robots.push_back(robot_scores[i].first);
      }
    }

    result[req.name] = assigned_robots;

    const std::unordered_set<uint8_t> assigned_set(assigned_robots.begin(), assigned_robots.end());
    std::erase_if(
      remaining_robots, [&assigned_set](uint8_t id) { return assigned_set.count(id) > 0; });

    RCLCPP_DEBUG_STREAM(
      logger_, "Session「" << req.name << "」に" << assigned_robots.size()
                           << "ロボットを割り当て: " << assigned_robots);
  }

  if (!remaining_robots.empty()) {
    RCLCPP_WARN(
      logger_, "%zu台のロボットがどのセッションにも割り当てられませんでした: %s",
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
