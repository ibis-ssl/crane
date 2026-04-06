// Copyright (c) 2025 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

/// mcap埋め込みメッセージ定義を使った動的デシリアライズのためのプレーンC++構造体。
/// ROSメッセージヘッダに依存しないため、定義が変わった古いrosbagも読み込める。

namespace crane::bag
{

// ─── 幾何プリミティブ ─────────────────────────────────────────────────────────

struct Point2D
{
  double x = 0, y = 0;
};

struct Point3D
{
  double x = 0, y = 0, z = 0;
};

struct Pose2D
{
  double x = 0, y = 0, theta = 0;
};

// ─── WorldModel ───────────────────────────────────────────────────────────────

struct BallInfo
{
  Point3D position;
  Point2D velocity;
};

struct RobotInfo
{
  uint8_t id = 0;
  Pose2D pose;
  Point2D velocity;
  bool available_vision = false;
};

struct FieldInfo
{
  double x = 0, y = 0;  // フィールド寸法 [m]
};

struct WorldModel
{
  BallInfo ball_info;
  FieldInfo field_info;
  bool is_yellow = false;
  std::vector<RobotInfo> robot_info_ours;
  std::vector<RobotInfo> robot_info_theirs;
};

// ─── PlaySituation ────────────────────────────────────────────────────────────

struct PlaySituation
{
  std::string command_name;
  std::string reason_text;
};

// ─── RobotCommands ────────────────────────────────────────────────────────────

struct NamedString
{
  std::string name;
  std::string value;
};

struct PositionTarget
{
  float target_x = 0, target_y = 0;
};

struct PolarVelocityTarget
{
  float target_velocity_r = 0, target_velocity_theta = 0;
};

struct RobotCommand
{
  uint8_t robot_id = 0;
  float kick_power = 0;
  float dribble_power = 0;
  bool stop_flag = false;
  bool chip_enable = false;
  std::string planner_name;
  std::vector<NamedString> planning_factors;
  std::vector<PositionTarget> position_target_mode;
  std::vector<PolarVelocityTarget> polar_velocity_target_mode;
};

struct RobotCommands
{
  std::vector<RobotCommand> robot_commands;
};

// ─── GameAnalysis ─────────────────────────────────────────────────────────────

struct GameAnalysis
{
  int32_t recommended_attacker_id = 0;
  float attacker_suitability_score = 0;
  int32_t pass_target_id = 0;
};

// ─── RobotSelectResults ───────────────────────────────────────────────────────

struct SelectResult
{
  std::string name;
  std::vector<uint8_t> selected_robots;
};

struct RobotSelectResults
{
  std::vector<SelectResult> results;
};

// ─── Log (rosout) ─────────────────────────────────────────────────────────────

struct LogMessage
{
  uint8_t level = 0;
  std::string name;
  std::string msg;
};

// ─── Referee ─────────────────────────────────────────────────────────────────

struct TeamInfo
{
  std::string name;
  uint32_t score = 0;
  uint32_t yellow_cards = 0;
  uint32_t red_cards = 0;
  uint32_t foul_counter = 0;
  uint32_t goalkeeper = 0;
};

/// ファウル種別ごとに必要なフィールドをフラット化して保持。
/// CDR上では全サブフィールドが存在するため、type_valueに応じて
/// 該当フィールドのみを読み取る（他フィールドはゼロ初期化）。
struct GameEventInfo
{
  int32_t type_value = 0;
  int32_t by_team_value = 0;
  uint32_t violator = 0;
  uint32_t victim = 0;
  uint32_t by_bot = 0;
  float location_x = 0, location_y = 0;
  float crash_speed = 0;
  float speed = 0;
  float duration = 0;
};

struct Referee
{
  int32_t command_value = 0;
  uint32_t command_counter = 0;
  int32_t stage_value = 0;
  TeamInfo yellow;
  TeamInfo blue;
  uint32_t has_field = 0;
  float designated_position_x = 0, designated_position_y = 0;
  int32_t next_command_value = 0;
  int32_t current_action_time_remaining = 0;
  std::vector<GameEventInfo> game_events;
};

// ─── Referee has_field ビットマスク定数 ───────────────────────────────────────
// robocup_ssl_msgs::msg::Referee と同じ値を持つ（IDLで定義）

constexpr uint32_t REFEREE_DESIGNATED_POSITION_FIELD_SET = 256;
constexpr uint32_t REFEREE_NEXT_COMMAND_FIELD_SET = 1024;

}  // namespace crane::bag
