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
// パケットサイズ
constexpr size_t PACKET_SIZE = 128;

// 受信バッファサイズ
constexpr size_t BUFFER_SIZE = 2048;
constexpr size_t TX_VALUE_COUNT = 14;

// バイトオフセット定義
namespace offset
{
constexpr int SYNC_0 = 0;
constexpr int SYNC_1 = 1;
constexpr int CHECKSUM = 2;
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

constexpr int CAMERA_POS_X_DIV2 = 60;
constexpr int CAMERA_POS_Y = 61;
constexpr int CAMERA_RADIUS_DIV4 = 62;
constexpr int CAMERA_FPS = 63;

constexpr int MOUSE_ODOM_X = 64;
constexpr int MOUSE_ODOM_Y = 68;
constexpr int MOUSE_VEL_X = 72;
constexpr int MOUSE_VEL_Y = 76;

constexpr int DEBUG_VALUES_START = 64;
constexpr int RESERVED_START = 120;
}  // namespace offset

// 定数値
constexpr float MOTOR_CURRENT_SCALE = 10.0f;
constexpr int KICK_STATE_SCALE = 10;
constexpr int FLOAT_SIZE = 4;
constexpr uint8_t SYNC_0_VALUE = 0xAB;
constexpr uint8_t SYNC_1_VALUE = 0xEA;

struct PacketValidationResult
{
  bool size_valid = false;
  bool sync_valid = false;
  bool checksum_valid = false;

  [[nodiscard]] auto valid() const -> bool { return size_valid && sync_valid && checksum_valid; }
};

template <typename BufferT>
inline auto readRawByte(const BufferT & buffer, size_t offset) -> uint8_t
{
  return static_cast<uint8_t>(buffer[offset]);
}

template <typename BufferT>
inline auto computeChecksum(const BufferT & buffer) -> uint8_t
{
  uint32_t checksum = 0;
  for (size_t i = offset::COUNTER; i < PACKET_SIZE; ++i) {
    checksum += readRawByte(buffer, i);
  }
  return static_cast<uint8_t>(checksum & 0xFF);
}

template <typename BufferT>
inline auto validatePacket(const BufferT & buffer) -> PacketValidationResult
{
  PacketValidationResult result;
  result.size_valid = buffer.size() == PACKET_SIZE;
  if (!result.size_valid) {
    return result;
  }

  result.sync_valid = readRawByte(buffer, offset::SYNC_0) == SYNC_0_VALUE &&
                      readRawByte(buffer, offset::SYNC_1) == SYNC_1_VALUE;
  result.checksum_valid = readRawByte(buffer, offset::CHECKSUM) == computeChecksum(buffer);
  return result;
}

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

  float feedback_age_ms = 0.0f;

  uint32_t valid_packet_count = 0;

  uint32_t invalid_packet_count = 0;

  uint32_t sync_error_count = 0;

  uint32_t checksum_error_count = 0;

  uint32_t size_mismatch_count = 0;

  uint32_t counter_jump_count = 0;

  uint8_t camera_pos_x_div2 = 0;

  uint8_t camera_pos_y = 0;

  uint8_t camera_radius_div4 = 0;

  uint8_t camera_fps = 0;

  uint8_t check_ver = 0;

  std::vector<float> values;
};

}  // namespace crane::robot_receiver

#endif  // CRANE_ROBOT_RECEIVER__ROBOT_FEEDBACK_PROTOCOL_HPP_
