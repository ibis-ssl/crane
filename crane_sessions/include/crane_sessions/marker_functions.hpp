// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__MARKER_FUNCTIONS_HPP_
#define CRANE_SESSIONS__MARKER_FUNCTIONS_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/crane_visualizer_wrapper.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <crane_robot_skills/marker.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace crane
{
/// 危険な敵ロボットをスコア付きで取得
/// @param world_model ワールドモデル
/// @return (敵ロボット, スコア) のペアのベクタ（スコア降順でソート済み）
auto getDangerEnemies(const WorldModelWrapper::SharedPtr & world_model)
  -> std::vector<std::pair<std::shared_ptr<RobotInfo>, double>>;

struct MarkingResult
{
  std::vector<std::shared_ptr<skills::Marker>> markers;
  std::vector<uint8_t> selected_robot_ids;
};

/// 危険な敵ロボットに対してマーカーを割り当てる共通関数
/// @param available_robot_ids 割り当て可能なロボットIDのリスト
/// @param world_model ワールドモデル
/// @param visualizer ビジュアライザー
/// @param command_name コマンド名（デバッグ用）
/// @param assign_remaining ターゲットがいない残余ロボットにもMarkerを生成するか
/// @param mark_mode マーキングモード ("intercept_pass": ボール基準, "save_goal": ゴール基準)
/// @return マーキング結果（生成したMarkerリストと選択されたロボットIDリスト）
auto assignMarkersToEnemies(
  const std::vector<uint8_t> & available_robot_ids,
  const WorldModelWrapper::SharedPtr & world_model,
  const VisualizerMessageBuilder::SharedPtr & visualizer, const std::string & command_name,
  bool assign_remaining = false, const std::string & mark_mode = "intercept_pass") -> MarkingResult;

}  // namespace crane
#endif  // CRANE_SESSIONS__MARKER_FUNCTIONS_HPP_
