// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// ゴールデンベクタの定義。C++ と Python が同じバイト列を作ることを確かめるための基準。
//
// ここではフィールドの並び（spec のキーと値）だけを持ち、RobotCommandV2 はそこから
// 組み立てる。構造体とフィールド表を別々に書くと両者が黙ってずれるので、片方だけにする。
//
// 覆えるのは「構造体で表現できる部分集合」だけ。未定義フラグビットや
// control_mode と mode_args の不一致は C++ 側に基準が存在しないので、
// Python の test_roundtrip.py が受け持つ。

#ifndef GOLDEN_VECTORS_HPP_
#define GOLDEN_VECTORS_HPP_

#include <crane_sender/robot_packet.h>

#include <string>
#include <vector>

namespace crane_packet_forge
{

struct FieldValue
{
  std::string key;  // spec / --set と同じ綴り
  double value;     // bool は 0/1、enum は整数値
};

struct GoldenVector
{
  std::string name;
  std::vector<FieldValue> fields;
};

/// spec の neutral base に相当する初期値。2 バイト値は 0.0 を符号化する。
inline auto neutralCommand() -> RobotCommandV2
{
  RobotCommandV2 command{};
  command.header = 0;
  command.check_counter = 0;
  command.vision_global_pos[0] = 0.0f;
  command.vision_global_pos[1] = 0.0f;
  command.vision_global_theta = 0.0f;
  command.is_vision_available = false;
  command.target_global_theta = 0.0f;
  command.kick_power = 0.0f;
  command.dribble_power = 0.0f;
  command.enable_chip = false;
  command.stop_emergency = false;
  command.acceleration_limit = 0.0f;
  command.linear_velocity_limit = 0.0f;
  command.angular_velocity_limit = 0.0f;
  command.latency_time_ms = 0;
  command.elapsed_time_ms_since_last_vision = 0;
  command.control_mode = POLAR_VELOCITY_TARGET_MODE;
  command.mode_args.polar_velocity.target_global_velocity_r = 0.0f;
  command.mode_args.polar_velocity.target_global_velocity_theta = 0.0f;
  command.target_global_pos[0] = 0.0f;
  command.target_global_pos[1] = 0.0f;
  command.terminal_velocity = 0.0f;
  return command;
}

/// spec のキーを RobotCommandV2 のメンバへ写す。未知のキーは false を返す。
inline auto applyField(RobotCommandV2 * command, const FieldValue & field) -> bool
{
  const auto & key = field.key;
  const auto value = static_cast<float>(field.value);

  if (key == "header") {
    command->header = static_cast<uint8_t>(field.value);
  } else if (key == "check_counter") {
    command->check_counter = static_cast<uint8_t>(field.value);
  } else if (key == "vision_global_pos.x") {
    command->vision_global_pos[0] = value;
  } else if (key == "vision_global_pos.y") {
    command->vision_global_pos[1] = value;
  } else if (key == "vision_global_theta") {
    command->vision_global_theta = value;
  } else if (key == "target_global_theta") {
    command->target_global_theta = value;
  } else if (key == "target_global_pos.x") {
    command->target_global_pos[0] = value;
  } else if (key == "target_global_pos.y") {
    command->target_global_pos[1] = value;
  } else if (key == "terminal_velocity") {
    command->terminal_velocity = value;
  } else if (key == "kick_power") {
    command->kick_power = value;
  } else if (key == "dribble_power") {
    command->dribble_power = value;
  } else if (key == "acceleration_limit") {
    command->acceleration_limit = value;
  } else if (key == "linear_velocity_limit") {
    command->linear_velocity_limit = value;
  } else if (key == "angular_velocity_limit") {
    command->angular_velocity_limit = value;
  } else if (key == "latency_time_ms") {
    command->latency_time_ms = static_cast<uint16_t>(field.value);
  } else if (key == "elapsed_time_ms_since_last_vision") {
    command->elapsed_time_ms_since_last_vision = static_cast<uint16_t>(field.value);
  } else if (key == "control_mode") {
    command->control_mode = static_cast<ControlMode>(static_cast<int>(field.value));
  } else if (key == "polar.target_global_velocity_r") {
    command->mode_args.polar_velocity.target_global_velocity_r = value;
  } else if (key == "polar.target_global_velocity_theta") {
    command->mode_args.polar_velocity.target_global_velocity_theta = value;
  } else if (key == "position_target.terminal_velocity_x") {
    command->mode_args.position_target.terminal_velocity_x = value;
  } else if (key == "position_target.terminal_velocity_y") {
    command->mode_args.position_target.terminal_velocity_y = value;
  } else if (key == "flags.is_vision_available") {
    command->is_vision_available = field.value != 0.0;
  } else if (key == "flags.enable_chip") {
    command->enable_chip = field.value != 0.0;
  } else if (key == "flags.stop_emergency") {
    command->stop_emergency = field.value != 0.0;
  } else {
    return false;
  }
  return true;
}

inline auto makeCommand(const std::vector<FieldValue> & fields) -> RobotCommandV2
{
  RobotCommandV2 command = neutralCommand();
  for (const auto & field : fields) {
    applyField(&command, field);
  }
  return command;
}

inline auto serializeToHex(const RobotCommandV2 & command) -> std::string
{
  // 【必須】ゼロ初期化する。serialize は union の非選択側 (byte 28-31) と
  // byte 38-63 に触らないので、初期化しないとスタックのゴミが混じる。
  RobotCommandSerializedV2 serialized{};
  RobotCommandSerializedV2_serialize(&serialized, &command);

  static const char * kHex = "0123456789abcdef";
  std::string out;
  out.reserve(sizeof(serialized.data) * 2);
  for (unsigned char byte : serialized.data) {
    out.push_back(kHex[byte >> 4]);
    out.push_back(kHex[byte & 0x0F]);
  }
  return out;
}

/// 2 バイトフィールドは全て「相異なる非ゼロ」の値で叩く。
/// 0 のままだとレンジの取り違え (±32.767 と ±M_PI) がこの照合をすり抜ける。
inline auto goldenVectors() -> std::vector<GoldenVector>
{
  return {
    {"neutral_defaults", {}},

    {"mode3_distinct",
     {
       {"header", 0},
       {"check_counter", 137},
       {"vision_global_pos.x", 1.234},
       {"vision_global_pos.y", -2.345},
       {"vision_global_theta", 0.75},
       {"target_global_theta", -1.25},
       {"target_global_pos.x", 3.456},
       {"target_global_pos.y", -4.567},
       {"terminal_velocity", 0.789},
       {"kick_power", 0.35},
       {"dribble_power", 0.85},
       {"acceleration_limit", 2.5},
       {"linear_velocity_limit", 3.25},
       {"angular_velocity_limit", 5.5},
       {"latency_time_ms", 300},
       {"elapsed_time_ms_since_last_vision", 421},
       {"control_mode", POLAR_VELOCITY_TARGET_MODE},
       {"polar.target_global_velocity_r", 1.75},
       {"polar.target_global_velocity_theta", -2.125},
       {"flags.is_vision_available", 1},
       {"flags.enable_chip", 1},
       {"flags.stop_emergency", 1},
     }},

    {"mode4_distinct",
     {
       {"check_counter", 200},
       {"vision_global_pos.x", -5.5},
       {"vision_global_pos.y", 6.25},
       {"vision_global_theta", -3.0},
       {"target_global_theta", 3.0},
       {"target_global_pos.x", -1.5},
       {"target_global_pos.y", 2.75},
       {"terminal_velocity", 1.125},
       {"kick_power", 1.0},
       {"dribble_power", 0.05},
       {"acceleration_limit", 8.75},
       {"linear_velocity_limit", 6.5},
       {"angular_velocity_limit", 12.25},
       {"latency_time_ms", 65535},
       {"elapsed_time_ms_since_last_vision", 501},
       {"control_mode", POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE},
       {"position_target.terminal_velocity_x", -0.875},
       {"position_target.terminal_velocity_y", 1.625},
       {"flags.is_vision_available", 1},
     }},

    // レンジの取り違えを突く。theta 系のうち vision/target は ±M_PI、
    // mode 3 の polar theta だけ ±32.767。同じ 1.0 を入れてもバイトが変わる。
    {"theta_range_probe",
     {
       {"vision_global_theta", 1.0},
       {"target_global_theta", 1.0},
       {"control_mode", POLAR_VELOCITY_TARGET_MODE},
       {"polar.target_global_velocity_r", 1.0},
       {"polar.target_global_velocity_theta", 1.0},
     }},

    // クランプの境界。convertFloatToTwoByte は値の側でクランプしてから量子化する。
    {"clamp_positive",
     {
       {"vision_global_pos.x", 100.0},
       {"vision_global_pos.y", 32.767},
       {"vision_global_theta", 6.28},
       {"target_global_theta", 3.141592653589793},
       {"target_global_pos.x", 32.767},
       {"target_global_pos.y", 40.0},
       {"terminal_velocity", 32.767},
       {"acceleration_limit", 32.767},
       {"linear_velocity_limit", 50.0},
       {"angular_velocity_limit", 32.767},
       {"polar.target_global_velocity_r", 32.767},
       {"polar.target_global_velocity_theta", 100.0},
     }},

    {"clamp_negative",
     {
       {"vision_global_pos.x", -100.0},
       {"vision_global_pos.y", -32.767},
       {"vision_global_theta", -6.28},
       {"target_global_theta", -3.141592653589793},
       {"target_global_pos.x", -32.767},
       {"target_global_pos.y", -40.0},
       {"terminal_velocity", -32.767},
       {"acceleration_limit", -32.767},
       {"linear_velocity_limit", -50.0},
       {"angular_velocity_limit", -32.767},
       {"polar.target_global_velocity_r", -32.767},
       {"polar.target_global_velocity_theta", -100.0},
     }},
  };
}

}  // namespace crane_packet_forge

#endif  // GOLDEN_VECTORS_HPP_
