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

// バイトオフセット定義。正本は ibis-ssl/specs の common/feedback_packet.md。
// 位置・速度・yaw などのバイト位置を旧定義（byte 4 から yaw が始まる配置）から読んではならない。
namespace offset
{
constexpr int SYNC_0 = 0;
constexpr int SYNC_1 = 1;
constexpr int CRC = 2;  // CRC-8/ATM。対象は byte 3..127
constexpr int COUNTER = 3;

// コア機能
constexpr int ERROR_ID = 5;
constexpr int ERROR_INFO = 7;
constexpr int ERROR_VALUE = 9;

// メイン基板
constexpr int YAW_ANGLE = 13;  // imu_yaw_deg
constexpr int BALL_DETECTION_0 = 17;
constexpr int BALL_DETECTION_1 = 18;
constexpr int BALL_DETECTION_2 = 19;  // ball_detection_extra
constexpr int DIFF_ANGLE = 20;

// 電源基板
constexpr int VOLTAGE_0 = 24;  // battery_voltage
constexpr int KICK_STATE = 28;
constexpr int TEMP_FET = 29;
constexpr int TEMP_COIL_0 = 30;
constexpr int TEMP_COIL_1 = 31;
constexpr int VOLTAGE_1 = 32;  // capacitor_boost_voltage
constexpr int MOUSE_ODOM_X = 36;
constexpr int MOUSE_ODOM_Y = 40;
constexpr int MOUSE_VEL_X = 44;
constexpr int MOUSE_VEL_Y = 48;
constexpr int MOUSE_QUALITY = 52;

// モーター基板
constexpr int MOTOR_CURRENT_0 = 56;
constexpr int TEMP_MOTOR_0 = 60;
constexpr int OUTPUT_VEL_X = 64;
constexpr int OUTPUT_VEL_Y = 68;
constexpr int MOTOR_FEEDBACK_0 = 72;
constexpr int LOCAL_ODOM_SPEED_MVF_X = 88;
constexpr int LOCAL_ODOM_SPEED_MVF_Y = 92;
constexpr int LOCAL_ODOM_SPEED_MVF_W = 96;

// 制御
constexpr int ODOM_X = 112;  // vision_based_position_x
constexpr int ODOM_Y = 116;
constexpr int ODOM_SPEED_X = 120;  // global_odom_speed_x
constexpr int ODOM_SPEED_Y = 124;
}  // namespace offset

// 定数値
constexpr float MOTOR_CURRENT_SCALE = 10.0f;
constexpr int KICK_STATE_SCALE = 10;
constexpr int FLOAT_SIZE = 4;
constexpr uint8_t SYNC_0_VALUE = 0xAB;
constexpr uint8_t SYNC_1_VALUE = 0xEA;

// CRC-8/ATM（多項式 0x07、初期値 0、入出力反転なし、最終 XOR 0）。"123456789" → 0xF4
template <typename BufferT>
inline auto crc8(const BufferT & buffer, size_t begin, size_t end) -> uint8_t
{
  uint8_t crc = 0;
  for (size_t i = begin; i < end; ++i) {
    crc ^= static_cast<uint8_t>(buffer[i]);
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<uint8_t>((crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1));
    }
  }
  return crc;
}

struct PacketValidationResult
{
  bool size_valid = false;
  bool sync_valid = false;
  bool crc_valid = false;

  [[nodiscard]] auto valid() const -> bool { return size_valid && sync_valid && crc_valid; }
};

template <typename BufferT>
inline auto readRawByte(const BufferT & buffer, size_t offset) -> uint8_t
{
  return static_cast<uint8_t>(buffer[offset]);
}

// received_size は今回の受信で実際に届いたバイト数。
// buffer は使い回しの受信バッファで BUFFER_SIZE (2048) 長のまま渡されるため、
// buffer.size() をパケット長と見なしてはならない。
template <typename BufferT>
inline auto validatePacket(const BufferT & buffer, size_t received_size) -> PacketValidationResult
{
  PacketValidationResult result;
  result.size_valid = received_size == PACKET_SIZE && buffer.size() >= PACKET_SIZE;
  if (!result.size_valid) {
    return result;
  }

  result.sync_valid = readRawByte(buffer, offset::SYNC_0) == SYNC_0_VALUE &&
                      readRawByte(buffer, offset::SYNC_1) == SYNC_1_VALUE;
  result.crc_valid = crc8(buffer, offset::COUNTER, PACKET_SIZE) == readRawByte(buffer, offset::CRC);
  return result;
}

// バッファ読み取りヘルパー関数 (std::memcpyを使用)
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

  // 新配置の ball_detection は 3 要素で、[3] は常に 0
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

  uint32_t crc_error_count = 0;

  uint32_t size_mismatch_count = 0;

  uint32_t counter_jump_count = 0;

  std::vector<float> values;
};

}  // namespace crane::robot_receiver

#endif  // CRANE_ROBOT_RECEIVER__ROBOT_FEEDBACK_PROTOCOL_HPP_
