// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <rosx_introspection/ros_parser.hpp>
#include <string>

#include "bag_types.hpp"

/// rosx_introspection の FlatMessage からプレーン構造体へ変換する関数群。
/// FlatMessage はトピック名をルートとしたフィールドパス→値のフラットなkey-valueリスト。
/// 例: "/world_model/ball_info/position/x" → 1.23
/// 配列要素は "/topic/array_field.N/subfield" の形式（N はゼロ始まりのインデックス）。

namespace crane::bag
{

WorldModel extract_world_model(const RosMsgParser::FlatMessage & flat);
PlaySituation extract_play_situation(const RosMsgParser::FlatMessage & flat);
RobotCommands extract_robot_commands(const RosMsgParser::FlatMessage & flat);
GameAnalysis extract_game_analysis(const RosMsgParser::FlatMessage & flat);
RobotSelectResults extract_robot_select_results(const RosMsgParser::FlatMessage & flat);
LogMessage extract_log_message(const RosMsgParser::FlatMessage & flat);
Referee extract_referee(const RosMsgParser::FlatMessage & flat);
KickPredictionTraceData extract_kick_prediction_trace(const RosMsgParser::FlatMessage & flat);

}  // namespace crane::bag
