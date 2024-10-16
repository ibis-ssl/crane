// Copyright (c) 2021 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_GAME_ANALYZER__EVALUATIONS__EVALUATIONS_HPP_
#define CRANE_GAME_ANALYZER__EVALUATIONS__EVALUATIONS_HPP_

#include <crane_basics/boost_geometry.hpp>
#include <crane_msg_wrappers/world_model_wrapper.hpp>

namespace crane::evaluation
{
/**
 * @brief 次の目標地点までのパスコースが敵ロボットに遮られていないかを評価する
 * @param p 評価する座標(ロボットがボールに触れてパスする地点)
 * @param next_target ボールを送り込む目標地点
 * @return 評価値(0~1)
 * @note 0 : パスカット可能性高（次のパスコースから敵が近い）
 * @note 1 : パスカット可能性低（次のパスコースから敵が遠い）
 */
double getNextTargetVisibleScore(
  Point p, Point next_target, WorldModelWrapper::SharedPtr world_model);

/**
 * @brief パス地点までの味方ロボットの到達性を評価する
 * @param p 評価する座標(ロボットがボールに触れてパスする地点)
 * @param nearest_dist 受け手ロボットから現在のパスコースへの最短距離
 * @return 評価値(0~1)
 * @note 0 : 到達性低(パス地点にパスロボットが遠い)
 * @note 1 : 到達性高(パス地点にパスロボットが近い)
 */
double getReachScore(
  RobotIdentifier id, Point p, double nearest_dist, WorldModelWrapper::SharedPtr world_model);

/**
 * @brief キック角度の難易度スコア（入射・反射角が大きいキックは難しい）
 * @param p 評価する座標(ロボットがボールに触れてパスする地点)
 * @param next_target next_target ボールを送り込む目標地点
 * @return 評価値(0~1)
 * @note 0 : キックが難しい(キック角度が大きい)
 * @note 1 : キックが簡単(キック角度が小さい)
 */
double getAngleScore(
  RobotIdentifier id, Point p, Point next_target, WorldModelWrapper::SharedPtr world_model);

/**
 * @brief パス地点の安全性を評価する（どれだけ敵ロボットから遠いか）
 * @param p 評価する座標(ロボットがボールに触れてパスする地点)
 * @param max_dist 考慮する最大距離（これ以上的ロボットが離れていれば評価値を1(安全）とする)
 * @return 評価値(0~1)
 * @note 0 : 危険(敵が近い)
 * @note 1 : 安全(敵が遠い)
 */
double getEnemyDistanceScore(
  Point p, WorldModelWrapper::SharedPtr world_model, double max_dist = 3.0);
}  // namespace crane::evaluation
#endif  // CRANE_GAME_ANALYZER__EVALUATIONS__EVALUATIONS_HPP_
