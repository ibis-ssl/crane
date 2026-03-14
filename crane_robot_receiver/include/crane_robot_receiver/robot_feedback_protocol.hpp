// Copyright (c) 2026 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_ROBOT_RECEIVER__ROBOT_FEEDBACK_PROTOCOL_HPP_
#define CRANE_ROBOT_RECEIVER__ROBOT_FEEDBACK_PROTOCOL_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <rclcpp/time.hpp>
#include <vector>

namespace crane::robot_receiver
{

// ロボットフィードバックプロトコル定数
namespace protocol
{
// バッファサイズ
constexpr size_t BUFFER_SIZE = 2048;
constexpr size_t DEBUG_VALUES_END = 128;

// バイトオフセット定義
namespace offset
{
constexpr int COUNTER = 3;

constexpr int YAW_ANGLE = 4;
constexpr int VOLTAGE_0 = 8;

constexpr int BALL_DETECTION_0 = 12;
constexpr int BALL_DETECTION_1 = 13;
constexpr int BALL_DETECTION_2 = 14;
constexpr int KICK_STATE = 15;

constexpr int ERROR_ID = 16;
constexpr int ERROR_INFO = 18;
constexpr int ERROR_VALUE = 20;

constexpr int MOTOR_CURRENT_0 = 24;
constexpr int MOTOR_CURRENT_1 = 25;
constexpr int MOTOR_CURRENT_2 = 26;
constexpr int MOTOR_CURRENT_3 = 27;

constexpr int BALL_DETECTION_3 = 28;

constexpr int TEMPERATURE_0 = 29;
constexpr int TEMPERATURE_1 = 30;
constexpr int TEMPERATURE_2 = 31;
constexpr int TEMPERATURE_3 = 32;
constexpr int TEMPERATURE_4 = 33;
constexpr int TEMPERATURE_5 = 34;
constexpr int TEMPERATURE_6 = 35;

constexpr int DIFF_ANGLE = 36;
constexpr int VOLTAGE_1 = 40;

constexpr int ODOM_X = 44;
constexpr int ODOM_Y = 48;
constexpr int ODOM_SPEED_X = 52;
constexpr int ODOM_SPEED_Y = 56;

constexpr int CHECK_VER = 60;

constexpr int MOUSE_ODOM_X = 64;
constexpr int MOUSE_ODOM_Y = 68;
constexpr int MOUSE_VEL_X = 72;
constexpr int MOUSE_VEL_Y = 76;

constexpr int DEBUG_VALUES_START = 64;
}  // namespace offset

// 定数値
constexpr float MOTOR_CURRENT_SCALE = 10.0f;
constexpr int KICK_STATE_SCALE = 10;
constexpr int FLOAT_SIZE = 4;

// バッファ読み取りヘルパー関数 (std::memcpyを使用)
inline auto readFloat(const std::vector<uint8_t> & buffer, int offset) -> float
{
  float value;
  std::memcpy(&value, &buffer[offset], sizeof(float));
  return value;
}

inline auto readUint16(const std::vector<uint8_t> & buffer, int offset) -> uint16_t
{
  uint16_t value;
  std::memcpy(&value, &buffer[offset], sizeof(uint16_t));
  return value;
}

inline auto readByte(const std::vector<uint8_t> & buffer, int offset) -> uint8_t
{
  return buffer[offset];
}

// std::vector<char> 版のオーバーロード
inline auto readFloat(const std::vector<char> & buffer, int offset) -> float
{
  float value;
  std::memcpy(&value, &buffer[offset], sizeof(float));
  return value;
}

inline auto readUint16(const std::vector<char> & buffer, int offset) -> uint16_t
{
  uint16_t value;
  std::memcpy(&value, &buffer[offset], sizeof(uint16_t));
  return value;
}

inline auto readByte(const std::vector<char> & buffer, int offset) -> uint8_t
{
  return static_cast<uint8_t>(buffer[offset]);
}
}  // namespace protocol

struct RobotFeedback
{
  rclcpp::Time received_stamp;

  uint8_t counter = 0;

  uint8_t kick_state = 0;

  uint8_t temperature[7] = {};

  uint16_t error_id = 0;

  uint16_t error_info = 0;

  float error_value = 0.0f;

  float motor_current[4] = {};

  uint8_t ball_detection[4] = {};

  bool ball_sensor = false;

  float yaw_angle = 0.0f;

  float diff_angle = 0.0f;

  std::array<float, 2> odom = {0.0f, 0.0f};

  std::array<float, 2> odom_speed = {0.0f, 0.0f};

  std::array<float, 2> mouse_odom = {0.0f, 0.0f};

  std::array<float, 2> mouse_vel = {0.0f, 0.0f};

  std::array<float, 2> voltage = {0.0f, 0.0f};

  uint8_t check_ver = 0;

  std::vector<float> values;
};

}  // namespace crane::robot_receiver

#endif  // CRANE_ROBOT_RECEIVER__ROBOT_FEEDBACK_PROTOCOL_HPP_
