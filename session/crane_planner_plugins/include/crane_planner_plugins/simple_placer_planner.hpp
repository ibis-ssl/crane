// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__SIMPLE_PLACER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__SIMPLE_PLACER_PLANNER_HPP_

#include <algorithm>
#include <crane_basics/boost_geometry.hpp>
#include <crane_basics/interval.hpp>
#include <crane_basics/position_assignments.hpp>
#include <crane_msg_wrappers/robot_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_msgs/srv/robot_select.hpp>
#include <crane_planner_base/planner_base.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "visibility_control.h"

namespace crane
{
struct AreaWithInfo
{
  std::string name;
  int our_robot_count;
  int their_robot_count;
  Box box;
};

template <typename K, typename V>
V getOr(const std::unordered_map<K, V> & map, const K & key, const V & value)
{
  auto it = map.find(key);
  if (it != map.end()) {
    return it->second;
  } else {
    return value;
  }
}

class SimplePlacerPlanner : public PlannerBase
{
private:
  std::vector<AreaWithInfo> defense_areas;

  std::unordered_map<uint8_t, std::string> assignment_map;

public:
  COMPOSITION_PUBLIC explicit SimplePlacerPlanner(
    WorldModelWrapper::SharedPtr & world_model,
    const ConsaiVisualizerWrapper::SharedPtr & visualizer)
  : PlannerBase("SimplePlacer", world_model, visualizer)
  {
    const double our_side_sign = world_model->getOurSideSign();
    {
      // FW
      Point p1, p2;
      p1 << -2.0 * our_side_sign, world_model->penalty_area_size.y() / 2.0;
      p2 << (world_model->field_size.x() * 0.5 - world_model->penalty_area_size.x()) * (-1.0) *
              (our_side_sign),
        -world_model->penalty_area_size.y() / 2.0;
      AreaWithInfo area;
      area.name = "FW";
      area.box = createBox(p1, p2);
      defense_areas.push_back(area);
    }
    {
      // MF
      Point p1, p2;
      p1 << 2.0, world_model->field_size.y() / 2.0;
      p2 << -2.0, -world_model->field_size.y() / 2.0;
      AreaWithInfo area;
      area.name = "MF";
      area.box = createBox(p1, p2);
      defense_areas.push_back(area);
    }
    {
      // RWG
      Point p1, p2;
      p1 << -2.0 * our_side_sign, world_model->field_size.y() / 2.0;
      p2 << world_model->field_size.x() * (-0.5) * our_side_sign,
        world_model->penalty_area_size.y() / 2.0;
      AreaWithInfo area;
      area.name = "WG1";
      area.box = createBox(p1, p2);
      defense_areas.push_back(area);
    }
    {
      // LWG
      Point p1, p2;
      p1 << -2.0 * our_side_sign, -world_model->penalty_area_size.y() / 2.0;
      p2 << world_model->field_size.x() * (-0.5) * our_side_sign,
        -world_model->field_size.y() / 2.0;
      AreaWithInfo area;
      area.name = "WG2";
      area.box = createBox(p1, p2);
      defense_areas.push_back(area);
    }
  }

  std::pair<Status, std::vector<crane_msgs::msg::RobotCommand>> calculateRobotCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    const auto & our_robots = world_model->ours.getAvailableRobots();
    const auto & their_robots = world_model->theirs.getAvailableRobots();
    // update defense area info
    for (auto & area_with_info : defense_areas) {
      area_with_info.our_robot_count = std::ranges::count_if(our_robots, [&](const auto & robot) {
        return isInBox(area_with_info.box, robot->pose.pos);
      });
      area_with_info.their_robot_count = std::ranges::count_if(
        their_robots,
        [&](const auto & robot) { return isInBox(area_with_info.box, robot->pose.pos); });
    }
    auto areas_with_info =
      defense_areas | ranges::views::transform([&](const auto & area_with_info) {
        int our_count = std::ranges::count_if(our_robots, [&](const auto & robot) {
          return isInBox(area_with_info.box, robot->pose.pos);
        });
        int their_count = std::ranges::count_if(their_robots, [&](const auto & robot) {
          return isInBox(area_with_info.box, robot->pose.pos);
        });
        return AreaWithInfo{area_with_info.name, our_count, their_count, area_with_info.box};
      }) |
      ranges::to<std::vector>();

    // sort by the number of robots in the area
    std::sort(areas_with_info.begin(), areas_with_info.end(), [](const auto & a, const auto & b) {
      return a.our_robot_count < b.our_robot_count;
    });

    auto robot_points = robots | ranges::views::transform([&](const auto & robot_id) {
                          return world_model->getRobot(robot_id)->pose.pos;
                        }) |
                        ranges::to<std::vector>();

    auto area_points = getAreaPoints(areas_with_info, robots.size());

    auto solution = getOptimalAssignments(robot_points, area_points);

    auto robot_commands =
      robots | ranges::views::transform([&, index = 0](const auto & robot_id) mutable {
        int current_area_robot_num = [&]() {
          if (assignment_map.contains(robot_id.robot_id)) {
            return ranges::find_if(
                     areas_with_info,
                     [&, name = assignment_map.at(robot_id.robot_id)](const auto & area_with_info) {
                       return area_with_info.name == name;
                     })
                     ->our_robot_count -
                   1;
          } else {
            // 多いと再配置される
            return 10;
          }
        }();

        Point target_pos;
        // current_area_robot_numよりロボットが少ないエリアを選ぶ（なければ同じ配置）
        const auto & area_with_info =
          ranges::find_if(areas_with_info, [&](const auto & area_with_info) {
            return area_with_info.our_robot_count < current_area_robot_num;
          });
        if (area_with_info != areas_with_info.end()) {
          assignment_map[robot_id.robot_id] = area_with_info->name;
          area_with_info->our_robot_count++;
          bg::centroid(area_with_info->box, target_pos);
        } else {
          // 同じエリアに配置
          // nameがassignment_map[robot_id.robot_id]と同じエリアを選ぶ
          if (const auto & area_with_info = ranges::find_if(
                areas_with_info,
                [&, name = assignment_map.at(robot_id.robot_id)](const auto & area_with_info) {
                  return area_with_info.name == name;
                });
              area_with_info != areas_with_info.end()) {
            bg::centroid(area_with_info->box, target_pos);
          }
        }
        auto command = std::make_shared<crane::RobotCommandWrapperPosition>(
          "simple_placer_planner", robot_id.robot_id, world_model);
        command->setTargetPosition(target_pos, 0.5).lookAtBallFrom(target_pos, 0.1);
        return command->getMsg();
      }) |
      ranges::to<std::vector>();

    // visualize areas with info
    for (const auto & area : areas_with_info) {
      visualizer->addRect(area.box, 1., "yellow", "", 1., area.name);
    }

    // viaulize robot assignments
    for (const auto & cmd : robot_commands) {
      visualizer->addLine(
        cmd.current_pose.x, cmd.current_pose.y, cmd.position_target_mode.front().target_x,
        cmd.position_target_mode.front().target_y, 1, "blue");
    }
    return {PlannerBase::Status::RUNNING, robot_commands};
  }

  auto getAreaPoints(const std::vector<AreaWithInfo> & areas, const std::size_t size)
    -> std::vector<Point>
  {
    std::vector<Point> area_points;
    for (const auto & area : areas) {
      Point p;
      bg::centroid(area.box, p);
      area_points.push_back(p);
      if (area_points.size() >= size) {
        break;
      }
    }
    return area_points;
  }

  auto getSelectedRobots(
    [[maybe_unused]] uint8_t selectable_robots_num, const std::vector<uint8_t> & selectable_robots,
    const std::unordered_map<uint8_t, RobotRole> & prev_roles) -> std::vector<uint8_t> override
  {
    assignment_map.clear();
    return this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [this](const std::shared_ptr<RobotInfo> & robot) {
        // choose id smaller first
        return 15. - static_cast<double>(-robot->id);
      },
      prev_roles);
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__SIMPLE_PLACER_PLANNER_HPP_
