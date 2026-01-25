// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SESSIONS__MARKER_FUNCTIONS_HPP_
#define CRANE_SESSIONS__MARKER_FUNCTIONS_HPP_

#include <crane_geometry/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace crane
{
/// 危険な敵ロボットをスコア付きで取得
/// @param world_model ワールドモデル
/// @return (敵ロボット, スコア) のペアのベクタ（スコア降順でソート済み）
auto getDangerEnemies(const WorldModelWrapper::SharedPtr & world_model)
  -> std::vector<std::pair<std::shared_ptr<RobotInfo>, double>>;

}  // namespace crane
#endif  // CRANE_SESSIONS__MARKER_FUNCTIONS_HPP_
