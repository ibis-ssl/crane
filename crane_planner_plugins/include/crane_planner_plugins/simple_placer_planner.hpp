// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_PLANNER_PLUGINS__SIMPLE_PLACER_PLANNER_HPP_
#define CRANE_PLANNER_PLUGINS__SIMPLE_PLACER_PLANNER_HPP_

#include <algorithm>
#include <crane_geometry/boost_geometry.hpp>
#include <crane_geometry/interval.hpp>
#include <crane_msg_wrappers/position_command_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_physics/position_assignments.hpp>
#include <crane_planner_plugins/planner_base.hpp>
#include <functional>
#include <memory>
#include <range/v3/algorithm/find_if.hpp>
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

  // ロボットIDからエリア名へのマッピング
  std::unordered_map<uint8_t, std::string> assignment_map;

  // ロボットの目標位置を保持
  std::unordered_map<uint8_t, Point> target_positions;

  // エリア移動の安定化のためのカウンター
  std::unordered_map<uint8_t, int> reassignment_cooldown;

  // 移動が完了したと判断する距離しきい値
  const double position_threshold = 0.15;

  // エリア再割り当ての待機フレーム数
  const int cooldown_frames = 1;

public:
  COMPOSITION_PUBLIC explicit SimplePlacerPlanner(
    WorldModelWrapper::SharedPtr & world_model, rclcpp::Node &)
  : PlannerBase("SimplePlacer", world_model)
  {
    const double our_side_sign = world_model->getOurSideSign();
    Point p1;
    Point p2;
    {
      // FW
      p1 << -2.0 * our_side_sign, world_model->penaltyAreaSize().y() / 2.0;
      p2 << (world_model->fieldSize().x() * 0.5 - world_model->penaltyAreaSize().x()) * -1.0 *
              our_side_sign,
        -world_model->penaltyAreaSize().y() / 2.0;
      AreaWithInfo area;
      area.name = "FW";
      area.box = createBox(p1, p2);
      defense_areas.push_back(area);
    }
    {
      // MF
      p1 << 2.0, world_model->fieldSize().y() / 2.0;
      p2 << -2.0, -world_model->fieldSize().y() / 2.0;
      AreaWithInfo area;
      area.name = "MF";
      area.box = createBox(p1, p2);
      defense_areas.push_back(area);
    }
    {
      // RWG
      p1 << -2.0 * our_side_sign, world_model->fieldSize().y() / 2.0;
      p2 << world_model->fieldSize().x() * -0.5 * our_side_sign,
        world_model->penaltyAreaSize().y() / 2.0;
      AreaWithInfo area;
      area.name = "WG1";
      area.box = createBox(p1, p2);
      defense_areas.push_back(area);
    }
    {
      // LWG
      p1 << -2.0 * our_side_sign, -world_model->penaltyAreaSize().y() / 2.0;
      p2 << world_model->fieldSize().x() * -0.5 * our_side_sign,
        -world_model->fieldSize().y() / 2.0;
      AreaWithInfo area;
      area.name = "WG2";
      area.box = createBox(p1, p2);
      defense_areas.push_back(area);
    }
  }

  std::pair<Status, std::vector<crane_msgs::msg::PositionCommand>> calculatePositionCommand(
    const std::vector<RobotIdentifier> & robots) override
  {
    const auto & our_robots = world_model->ours().getAvailableRobots();
    const auto & their_robots = world_model->theirs().getAvailableRobots();
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
    ranges::sort(areas_with_info, [](const auto & a, const auto & b) {
      return a.their_robot_count < b.their_robot_count;
    });

    auto robot_commands =
      robots | ranges::views::transform([&, index = 0](const auto & robot_id) mutable {
        // クールダウンカウンターの更新
        if (
          reassignment_cooldown.find(robot_id.id) != reassignment_cooldown.end() &&
          reassignment_cooldown[robot_id.id] > 0) {
          reassignment_cooldown[robot_id.id]--;
        }

        // 現在の目標位置と実際のロボット位置間の距離を計算
        double distance_to_target = 100.0;  // 初期値として大きな値を設定
        if (target_positions.find(robot_id.id) != target_positions.end()) {
          distance_to_target =
            (world_model->getOurRobot(robot_id.id)->pose.pos - target_positions[robot_id.id])
              .norm();
        }

        // 目標位置に到達していて、かつクールダウンが0の場合のみ再割り当てを検討
        bool should_reassign =
          distance_to_target < position_threshold &&
          (reassignment_cooldown.find(robot_id.id) == reassignment_cooldown.end() ||
           reassignment_cooldown[robot_id.id] == 0);

        Point target_pos;

        // 現在のエリアにおけるロボット数（自分を除く）を取得
        int current_area_robot_num = [&]() {
          if (assignment_map.contains(robot_id.id)) {
            // 自分のエリアは自分を除いてカウントする
            auto it = ranges::find_if(
              areas_with_info,
              [&, name = assignment_map.at(robot_id.id)](const auto & area_with_info) {
                return area_with_info.name == name;
              });
            if (it != areas_with_info.end()) {
              return it->our_robot_count - 1;
            }
          }
          // 初期割り当て時や問題発生時は大きな値を返す
          return 10;
        }();

        if (should_reassign) {
          // より少ないロボットがいるエリアを選択する
          const auto & area_with_info =
            ranges::find_if(areas_with_info, [current_area_robot_num](const auto & area_with_info) {
              return area_with_info.our_robot_count < current_area_robot_num;
            });

          if (area_with_info != areas_with_info.end()) {
            // 新しいエリアへの割り当て
            assignment_map[robot_id.id] = area_with_info->name;
            area_with_info->our_robot_count++;
            bg::centroid(area_with_info->box, target_pos);

            // 目標位置更新と再割り当てのクールダウンを設定
            target_positions[robot_id.id] = target_pos;
            reassignment_cooldown[robot_id.id] = cooldown_frames;
          } else {
            // 同じエリアに留まる
            if (assignment_map.find(robot_id.id) != assignment_map.end()) {
              const auto & same_area = ranges::find_if(
                areas_with_info, [&, name = assignment_map.at(robot_id.id)](const auto & area) {
                  return area.name == name;
                });

              if (same_area != areas_with_info.end()) {
                bg::centroid(same_area->box, target_pos);
                target_positions[robot_id.id] = target_pos;
              }
            }
          }
        } else if (target_positions.find(robot_id.id) != target_positions.end()) {
          // 既に目標位置がある場合はそれを使用
          target_pos = target_positions[robot_id.id];
        } else {
          // 初期割り当て時など、現在位置を目標とする
          target_pos = world_model->getOurRobot(robot_id.id)->pose.pos;
          target_positions[robot_id.id] = target_pos;
        }

        auto command = std::make_shared<crane::PositionCommandWrapper>(
          "simple_placer_planner", robot_id.id, world_model);

        // 目標位置と角度の設定
        command->setTargetPosition(target_pos, 0.1).lookAtBallFrom(target_pos, 0.1);

        return command->getMsg();
      }) |
      ranges::to<std::vector>();

    // エリアの可視化
    for (const auto & area : areas_with_info) {
      visualizer->rect().box(area.box).stroke("yellow").strokeWidth(10).build();
      visualizer->text()
        .position(area.box.min_corner().x(), area.box.min_corner().y())
        .text(area.name)
        .fill("yellow")
        .fontSize(100)
        .build();
    }

    // ロボットの移動ラインを可視化
    for (const auto & cmd : robot_commands) {
      visualizer->line()
        .start(cmd.current_pose.x, cmd.current_pose.y)
        .end(cmd.target_x, cmd.target_y)
        .stroke("blue")
        .strokeWidth(10)
        .build();

      // 目標到達状態を可視化
      uint8_t robot_id = cmd.robot_id;
      double distance =
        (Point(cmd.current_pose.x, cmd.current_pose.y) - Point(cmd.target_x, cmd.target_y)).norm();

      std::string circle_color = (distance < position_threshold) ? "green" : "red";
      visualizer->circle()
        .center(cmd.target_x, cmd.target_y)
        .radius(0.1)
        .fill(circle_color, 0.5)
        .build();
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
    // 前回からの割り当て情報を維持
    std::vector<uint8_t> selected = this->getSelectedRobotsByScore(
      selectable_robots_num, selectable_robots,
      [](const std::shared_ptr<RobotInfo> & robot) {
        // choose id smaller first
        return 15. - static_cast<double>(-robot->id);
      },
      prev_roles);

    // 新しく選択されたロボットだけを初期化
    for (const auto & robot_id : selected) {
      if (assignment_map.find(robot_id) == assignment_map.end()) {
        // 初期割り当てを空に
        Point current_pos = world_model->getOurRobot(robot_id)->pose.pos;

        // 現在位置から最も近いエリアを選択
        double min_distance = std::numeric_limits<double>::max();
        std::string closest_area = "";

        for (const auto & area : defense_areas) {
          Point centroid;
          bg::centroid(area.box, centroid);
          double distance = (current_pos - centroid).norm();

          if (distance < min_distance) {
            min_distance = distance;
            closest_area = area.name;
          }
        }

        if (!closest_area.empty()) {
          assignment_map[robot_id] = closest_area;
        }
      }
    }

    // 選択から外れたロボットの情報をクリア
    for (auto it = assignment_map.begin(); it != assignment_map.end();) {
      if (std::find(selected.begin(), selected.end(), it->first) == selected.end()) {
        it = assignment_map.erase(it);
        target_positions.erase(it->first);
        reassignment_cooldown.erase(it->first);
      } else {
        ++it;
      }
    }

    return selected;
  }
};

}  // namespace crane
#endif  // CRANE_PLANNER_PLUGINS__SIMPLE_PLACER_PLANNER_HPP_
