// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER_ROBOT_PACKET_HPP
#define CRANE_LOCAL_PLANNER_ROBOT_PACKET_HPP
#include <cmath>

auto convertFloatToTwoByte(float val, float range) -> std::pair<uint8_t, uint8_t>
{
  uint16_t two_byte = static_cast<int>(32767 * static_cast<float>(val / range) + 32767);
  uint8_t byte_low, byte_high;
  byte_low = two_byte & 0x00FF;
  byte_high = (two_byte & 0xFF00) >> 8;
  return std::make_pair(byte_high, byte_low);
}

auto convertTwoByteToFloat(uint8_t byte_high, uint8_t byte_low, float range) -> float
{
  uint16_t two_byte = (byte_high << 8) | byte_low;
  float val = static_cast<float>(two_byte - 32767) / 32767 * range;
  return val;
}

struct RobotCommandSerialized;

struct RobotCommand
{
  uint8_t HEADER;
  uint8_t CHECK;
  float VEL_LOCAL_SURGE;
  float VEL_LOCAL_SWAY;
  float VISION_GLOBAL_X;
  float VISION_GLOBAL_Y;
  float VISION_GLOBAL_THETA;
  float TARGET_GLOBAL_X;
  float TARGET_GLOBAL_Y;
  float BALL_GLOBAL_X;
  float BALL_GLOBAL_Y;
  float TARGET_GLOBAL_THETA;

  bool LOCAL_FEEDBACK_ENABLE;
  bool LOCAL_KEEPER_MODE_ENABLE;
  bool IS_ID_VISIBLE;
  bool STOP_FLAG;
  bool IS_DRIBBLER_UP;
  float KICK_POWER;
  float DRIBBLE_POWER;
  bool CHIP_ENABLE;
  operator RobotCommandSerialized() const;
};

struct RobotCommandSerialized
{
  enum class Address {
    //    HEADER,
    CHECK,
    VEL_LOCAL_SURGE_HIGH,
    VEL_LOCAL_SURGE_LOW,
    VEL_LOCAL_SWAY_HIGH,
    VEL_LOCAL_SWAY_LOW,
    VISION_GLOBAL_THETA_HIGH,
    VISION_GLOBAL_THETA_LOW,
    TARGET_GLOBAL_THETA_HIGH,
    TARGET_GLOBAL_THETA_LOW,
    KICK_POWER,
    DRIBBLE_POWER,
    LOCAL_FLAGS,
    BALL_GLOBAL_X_HIGH,
    BALL_GLOBAL_X_LOW,
    BALL_GLOBAL_Y_HIGH,
    BALL_GLOBAL_Y_LOW,
    VISION_GLOBAL_X_HIGH,
    VISION_GLOBAL_X_LOW,
    VISION_GLOBAL_Y_HIGH,
    VISION_GLOBAL_Y_LOW,
    TARGET_GLOBAL_X_HIGH,
    TARGET_GLOBAL_X_LOW,
    TARGET_GLOBAL_Y_HIGH,
    TARGET_GLOBAL_Y_LOW,
    SIZE
  };

  operator RobotCommand() const
  {
    RobotCommand packet;

#define FLOAT_FROM_2BYTE(name, range)                                                          \
  packet.name = convertTwoByteToFloat(                                                         \
    data[static_cast<int>(Address::name##_HIGH)], data[static_cast<int>(Address::name##_LOW)], \
    range)

#define FLOAT_FROM_1BYTE(name, range) \
  packet.name = data[static_cast<int>(Address::name)] / 255.0f * range

    //    packet.HEADER = data[static_cast<int>(Address::HEADER)];
    packet.CHECK = data[static_cast<int>(Address::CHECK)];
    FLOAT_FROM_2BYTE(VEL_LOCAL_SURGE, 7.0);
    FLOAT_FROM_2BYTE(VEL_LOCAL_SWAY, 7.0);
    FLOAT_FROM_2BYTE(VISION_GLOBAL_X, 32.767);
    FLOAT_FROM_2BYTE(VISION_GLOBAL_Y, 32.767);
    FLOAT_FROM_2BYTE(VISION_GLOBAL_THETA, M_PI);
    FLOAT_FROM_2BYTE(TARGET_GLOBAL_X, 32.767);
    FLOAT_FROM_2BYTE(TARGET_GLOBAL_Y, 32.767);
    FLOAT_FROM_2BYTE(BALL_GLOBAL_X, 32.767);
    FLOAT_FROM_2BYTE(BALL_GLOBAL_Y, 32.767);
    FLOAT_FROM_2BYTE(TARGET_GLOBAL_THETA, M_PI);
    const uint8_t kick_raw = data[static_cast<int>(Address::KICK_POWER)];
    if (kick_raw > 100) {
      packet.CHIP_ENABLE = true;
      packet.KICK_POWER = (kick_raw - 100) / 20.0f;
    } else {
      packet.CHIP_ENABLE = false;
      packet.KICK_POWER = kick_raw / 20.0f;
    }
    packet.DRIBBLE_POWER = data[static_cast<int>(Address::DRIBBLE_POWER)] / 20.0f;

    uint8_t local_flags = data[static_cast<int>(Address::LOCAL_FLAGS)];
    packet.LOCAL_FEEDBACK_ENABLE = local_flags & 0x04;
    packet.LOCAL_KEEPER_MODE_ENABLE = local_flags & 0x10;
    packet.IS_ID_VISIBLE = local_flags & 0x01;

#undef FLOAT_FROM_1BYTE
#undef FLOAT_FROM_2BYTE

    return packet;
  }

  uint8_t data[static_cast<int>(Address::SIZE)];
};

RobotCommand::operator RobotCommandSerialized() const
{
  RobotCommandSerialized serialized;

#define FLOAT_TO_2BYTE(name, range)                                                 \
  std::pair<uint8_t, uint8_t> name##_two_byte = convertFloatToTwoByte(name, range); \
  serialized.data[static_cast<int>(RobotCommandSerialized::Address::name##_HIGH)] = \
    name##_two_byte.first;                                                          \
  serialized.data[static_cast<int>(RobotCommandSerialized::Address::name##_LOW)] =  \
    name##_two_byte.second

#define FLOAT_TO_1BYTE(name, range)                                      \
  uint8_t name##_one_byte = static_cast<uint8_t>(name / range * 255.0f); \
  serialized.data[static_cast<int>(RobotCommandSerialized::Address::name)] = name##_one_byte

  //  serialized.data[static_cast<int>(RobotCommandSerialized::Address::HEADER)] = HEADER;
  serialized.data[static_cast<int>(RobotCommandSerialized::Address::CHECK)] = CHECK;

  FLOAT_TO_2BYTE(VEL_LOCAL_SURGE, 7.0);
  FLOAT_TO_2BYTE(VEL_LOCAL_SWAY, 7.0);
  FLOAT_TO_2BYTE(VISION_GLOBAL_X, 32.767);
  FLOAT_TO_2BYTE(VISION_GLOBAL_Y, 32.767);
  FLOAT_TO_2BYTE(VISION_GLOBAL_THETA, M_PI);
  FLOAT_TO_2BYTE(TARGET_GLOBAL_X, 32.767);
  FLOAT_TO_2BYTE(TARGET_GLOBAL_Y, 32.767);
  FLOAT_TO_2BYTE(BALL_GLOBAL_X, 32.767);
  FLOAT_TO_2BYTE(BALL_GLOBAL_Y, 32.767);
  FLOAT_TO_2BYTE(TARGET_GLOBAL_THETA, M_PI);
  serialized.data[static_cast<int>(RobotCommandSerialized::Address::DRIBBLE_POWER)] =
    static_cast<uint8_t>(DRIBBLE_POWER * 20);
  serialized.data[static_cast<int>(RobotCommandSerialized::Address::KICK_POWER)] =
    [&]() -> uint8_t {
    if (CHIP_ENABLE) {
      return static_cast<uint8_t>((std::round(20 * KICK_POWER) + 100));
    } else {
      return static_cast<uint8_t>(std::round(20 * KICK_POWER));
    }
  }();

  uint8_t local_flags = 0x00;
  local_flags |= (IS_ID_VISIBLE << 0);
  local_flags |= (LOCAL_KEEPER_MODE_ENABLE << 1);
  local_flags |= (STOP_FLAG << 2);
  local_flags |= (LOCAL_FEEDBACK_ENABLE << 3);
  local_flags |= (IS_DRIBBLER_UP << 4);

  serialized.data[static_cast<int>(RobotCommandSerialized::Address::LOCAL_FLAGS)] = local_flags;

#undef FLOAT_TO_1BYTE
#undef FLOAT_TO_2BYTE

  return serialized;
}

#endif  //CRANE_LOCAL_PLANNER_ROBOT_PACKET_HPP
