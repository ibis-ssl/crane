// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_LOCAL_PLANNER_ROBOT_PACKET_HPP
#define CRANE_LOCAL_PLANNER_ROBOT_PACKET_HPP

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
  float KICK_POWER;
  float DRIBBLE_POWER;
  operator RobotCommandSerialized() const;
};

struct RobotCommandSerialized
{
  enum class Address {
    HEADER,
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
    LOCAL_KEEPER_MODE_ENABLE,
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
    LOCAL_FEEDBACK_MODE_ENABLE,
    NUM
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

    packet.HEADER = data[static_cast<int>(Address::HEADER)];
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
    packet.LOCAL_FEEDBACK_ENABLE = data[static_cast<int>(Address::LOCAL_FEEDBACK_MODE_ENABLE)];
    packet.LOCAL_KEEPER_MODE_ENABLE = data[static_cast<int>(Address::LOCAL_KEEPER_MODE_ENABLE)];
    FLOAT_FROM_1BYTE(KICK_POWER, 1.0);
    FLOAT_FROM_1BYTE(DRIBBLE_POWER, 1.0);

#undef FLOAT_FROM_1BYTE
#undef FLOAT_FROM_2BYTE

    return packet;
  }

  uint8_t data[static_cast<int>(Address::NUM)];
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

#define FLOAT_TO_1BYTE(name, range)                                             \
  uint8_t name##_one_byte = static_cast<uint8_t>(name / range * 255.0f); \
  serialized.data[static_cast<int>(RobotCommandSerialized::Address::name)] = name##_one_byte

  serialized.data[static_cast<int>(RobotCommandSerialized::Address::HEADER)] = HEADER;
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
  serialized.data[static_cast<int>(RobotCommandSerialized::Address::LOCAL_FEEDBACK_MODE_ENABLE)] =
    LOCAL_FEEDBACK_ENABLE;
  serialized.data[static_cast<int>(RobotCommandSerialized::Address::LOCAL_KEEPER_MODE_ENABLE)] =
    LOCAL_KEEPER_MODE_ENABLE;
  FLOAT_TO_1BYTE(KICK_POWER, 1.0);
  FLOAT_TO_1BYTE(DRIBBLE_POWER, 1.0);

#undef FLOAT_TO_1BYTE
#undef FLOAT_TO_2BYTE

  return serialized;
}

#endif  //CRANE_LOCAL_PLANNER_ROBOT_PACKET_HPP
