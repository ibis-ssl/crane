// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__ROBOT_PACKET_H_
#define CRANE_SENDER__ROBOT_PACKET_H_

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <iostream>
#include <ostream>

// NOLINTBEGIN(readability/casting)

typedef struct
{
  uint8_t high;
  uint8_t low;
} TwoByte;

inline TwoByte convertFloatToTwoByte(float val, float range)
{
  if (val > range) {
    val = range;
    std::cout << "Warning: value is out of range(overflow)" << std::endl;
  } else if (val < -range) {
    val = -range;
    std::cout << "Warning: value is out of range(underflow)" << std::endl;
  }
  TwoByte result;
  uint16_t uint16 = (uint16_t)(32767.f * (float)(val / range) + 32767.f);
  result.high = (uint16 & 0xFF00) >> 8;
  result.low = uint16 & 0x00FF;
  return result;
}

inline float convertTwoByteToFloat(uint8_t byte_high, uint8_t byte_low, float range)
{
  uint16_t two_byte = (byte_high << 8) | byte_low;
  return (float)(two_byte - 32767.f) / 32767.f * range;
}

inline TwoByte convertUInt16ToTwoByte(uint16_t val)
{
  TwoByte result;
  result.high = (val & 0xFF00) >> 8;
  result.low = val & 0x00FF;
  return result;
}

inline uint16_t convertTwoByteToUInt16(uint8_t byte_high, uint8_t byte_low)
{
  return (byte_high << 8) | byte_low;
}

inline void forward(uint8_t * arg1, uint8_t * arg2, float val, float range)
{
  TwoByte two_byte = convertFloatToTwoByte(val, range);
  *arg1 = two_byte.high;
  *arg2 = two_byte.low;
}

#define MODE_ARGS_SIZE (8)

typedef struct
{
  float target_global_velocity_r;
  float target_global_velocity_theta;
} PolarVelocityModeArgs;

inline void PolarVelocityModeArgs_init(PolarVelocityModeArgs * args, const uint8_t * data)
{
  args->target_global_velocity_r = convertTwoByteToFloat(data[0], data[1], 32.767);
  args->target_global_velocity_theta = convertTwoByteToFloat(data[2], data[3], 32.767);
}

inline void PolarVelocityModeArgs_serialize(const PolarVelocityModeArgs * args, uint8_t * data)
{
  forward(&data[0], &data[1], args->target_global_velocity_r, 32.767);
  forward(&data[2], &data[3], args->target_global_velocity_theta, 32.767);
}

typedef enum {
  POLAR_VELOCITY_TARGET_MODE = 3,
} ControlMode;

typedef struct
{
  uint8_t header;
  uint8_t check_counter;

  float vision_global_pos[2];
  float vision_global_theta;
  bool is_vision_available;
  float target_global_theta;
  float kick_power;
  float dribble_power;
  bool enable_chip;
  bool stop_emergency;
  float acceleration_limit;
  float linear_velocity_limit;
  float angular_velocity_limit;
  uint16_t latency_time_ms;
  uint16_t elapsed_time_ms_since_last_vision;
  ControlMode control_mode;

  union {
    PolarVelocityModeArgs polar_velocity;
  } mode_args;

  float target_global_pos[2];
  float terminal_velocity;
} RobotCommandV2;

typedef struct
{
  uint8_t data[64];
} RobotCommandSerializedV2;

enum Address {
  HEADER,
  CHECK_COUNTER,
  VISION_GLOBAL_X_HIGH,
  VISION_GLOBAL_X_LOW,
  VISION_GLOBAL_Y_HIGH,
  VISION_GLOBAL_Y_LOW,
  VISION_GLOBAL_THETA_HIGH,
  VISION_GLOBAL_THETA_LOW,
  TARGET_GLOBAL_THETA_HIGH,
  TARGET_GLOBAL_THETA_LOW,
  KICK_POWER,
  DRIBBLE_POWER,
  ACCELERATION_LIMIT_HIGH,
  ACCELERATION_LIMIT_LOW,
  LINEAR_VELOCITY_LIMIT_HIGH,
  LINEAR_VELOCITY_LIMIT_LOW,
  ANGULAR_VELOCITY_LIMIT_HIGH,
  ANGULAR_VELOCITY_LIMIT_LOW,
  LATENCY_TIME_MS_HIGH,
  LATENCY_TIME_MS_LOW,
  ELAPSED_TIME_MS_SINCE_LAST_VISION_HIGH,
  ELAPSED_TIME_MS_SINCE_LAST_VISION_LOW,
  FLAGS,
  CONTROL_MODE,
  CONTROL_MODE_ARGS,
  TARGET_GLOBAL_POS_X_HIGH = CONTROL_MODE_ARGS + MODE_ARGS_SIZE,
  TARGET_GLOBAL_POS_X_LOW,
  TARGET_GLOBAL_POS_Y_HIGH,
  TARGET_GLOBAL_POS_Y_LOW,
  TERMINAL_VELOCITY_HIGH,
  TERMINAL_VELOCITY_LOW,
};

enum FlagAddress {
  IS_VISION_AVAILABLE = 0,
  ENABLE_CHIP = 1,
  STOP_EMERGENCY = 3,
};

inline void RobotCommandSerializedV2_serialize(
  RobotCommandSerializedV2 * serialized, const RobotCommandV2 * command)
{
  serialized->data[HEADER] = command->header;
  serialized->data[CHECK_COUNTER] = command->check_counter;
  forward(
    &serialized->data[VISION_GLOBAL_X_HIGH], &serialized->data[VISION_GLOBAL_X_LOW],
    command->vision_global_pos[0], 32.767);
  forward(
    &serialized->data[VISION_GLOBAL_Y_HIGH], &serialized->data[VISION_GLOBAL_Y_LOW],
    command->vision_global_pos[1], 32.767);
  forward(
    &serialized->data[VISION_GLOBAL_THETA_HIGH], &serialized->data[VISION_GLOBAL_THETA_LOW],
    command->vision_global_theta, M_PI);
  forward(
    &serialized->data[TARGET_GLOBAL_THETA_HIGH], &serialized->data[TARGET_GLOBAL_THETA_LOW],
    command->target_global_theta, M_PI);
  serialized->data[KICK_POWER] = command->kick_power * 20;
  serialized->data[DRIBBLE_POWER] = command->dribble_power * 20;
  forward(
    &serialized->data[ACCELERATION_LIMIT_HIGH], &serialized->data[ACCELERATION_LIMIT_LOW],
    command->acceleration_limit, 32.767);
  forward(
    &serialized->data[LINEAR_VELOCITY_LIMIT_HIGH], &serialized->data[LINEAR_VELOCITY_LIMIT_LOW],
    command->linear_velocity_limit, 32.767);
  forward(
    &serialized->data[ANGULAR_VELOCITY_LIMIT_HIGH], &serialized->data[ANGULAR_VELOCITY_LIMIT_LOW],
    command->angular_velocity_limit, 32.767);
  TwoByte latency_time = convertUInt16ToTwoByte(command->latency_time_ms);
  serialized->data[LATENCY_TIME_MS_HIGH] = latency_time.high;
  serialized->data[LATENCY_TIME_MS_LOW] = latency_time.low;
  TwoByte elapsed_time = convertUInt16ToTwoByte(command->elapsed_time_ms_since_last_vision);
  serialized->data[ELAPSED_TIME_MS_SINCE_LAST_VISION_HIGH] = elapsed_time.high;
  serialized->data[ELAPSED_TIME_MS_SINCE_LAST_VISION_LOW] = elapsed_time.low;
  uint8_t flags = 0x00;
  flags |= (command->is_vision_available << IS_VISION_AVAILABLE);
  flags |= (command->enable_chip << ENABLE_CHIP);
  flags |= (command->stop_emergency << STOP_EMERGENCY);
  serialized->data[FLAGS] = flags;
  serialized->data[CONTROL_MODE] = (uint8_t)command->control_mode;
  switch (command->control_mode) {
    case POLAR_VELOCITY_TARGET_MODE:
      PolarVelocityModeArgs_serialize(
        &command->mode_args.polar_velocity, &serialized->data[CONTROL_MODE_ARGS]);
      break;
  }
  forward(
    &serialized->data[TARGET_GLOBAL_POS_X_HIGH], &serialized->data[TARGET_GLOBAL_POS_X_LOW],
    command->target_global_pos[0], 32.767);
  forward(
    &serialized->data[TARGET_GLOBAL_POS_Y_HIGH], &serialized->data[TARGET_GLOBAL_POS_Y_LOW],
    command->target_global_pos[1], 32.767);
  forward(
    &serialized->data[TERMINAL_VELOCITY_HIGH], &serialized->data[TERMINAL_VELOCITY_LOW],
    command->terminal_velocity, 32.767);
}

inline RobotCommandV2 RobotCommandSerializedV2_deserialize(
  const RobotCommandSerializedV2 * serialized)
{
  RobotCommandV2 command;
  command.header = serialized->data[HEADER];
  command.check_counter = serialized->data[CHECK_COUNTER];
  command.vision_global_pos[0] = convertTwoByteToFloat(
    serialized->data[VISION_GLOBAL_X_HIGH], serialized->data[VISION_GLOBAL_X_LOW], 32.767);
  command.vision_global_pos[1] = convertTwoByteToFloat(
    serialized->data[VISION_GLOBAL_Y_HIGH], serialized->data[VISION_GLOBAL_Y_LOW], 32.767);
  command.vision_global_theta = convertTwoByteToFloat(
    serialized->data[VISION_GLOBAL_THETA_HIGH], serialized->data[VISION_GLOBAL_THETA_LOW], M_PI);
  command.target_global_theta = convertTwoByteToFloat(
    serialized->data[TARGET_GLOBAL_THETA_HIGH], serialized->data[TARGET_GLOBAL_THETA_LOW], M_PI);
  command.kick_power = serialized->data[KICK_POWER] / 20.;
  command.dribble_power = serialized->data[DRIBBLE_POWER] / 20.;
  command.acceleration_limit = convertTwoByteToFloat(
    serialized->data[ACCELERATION_LIMIT_HIGH], serialized->data[ACCELERATION_LIMIT_LOW], 32.767);
  command.linear_velocity_limit = convertTwoByteToFloat(
    serialized->data[LINEAR_VELOCITY_LIMIT_HIGH], serialized->data[LINEAR_VELOCITY_LIMIT_LOW],
    32.767);
  command.angular_velocity_limit = convertTwoByteToFloat(
    serialized->data[ANGULAR_VELOCITY_LIMIT_HIGH], serialized->data[ANGULAR_VELOCITY_LIMIT_LOW],
    32.767);
  command.latency_time_ms = convertTwoByteToUInt16(
    serialized->data[LATENCY_TIME_MS_HIGH], serialized->data[LATENCY_TIME_MS_LOW]);
  command.elapsed_time_ms_since_last_vision = convertTwoByteToUInt16(
    serialized->data[ELAPSED_TIME_MS_SINCE_LAST_VISION_HIGH],
    serialized->data[ELAPSED_TIME_MS_SINCE_LAST_VISION_LOW]);
  uint8_t flags = serialized->data[FLAGS];
  command.is_vision_available = (flags >> IS_VISION_AVAILABLE) & 0x01;
  command.enable_chip = (flags >> ENABLE_CHIP) & 0x01;
  command.stop_emergency = (flags >> STOP_EMERGENCY) & 0x01;
  command.control_mode = (ControlMode)serialized->data[CONTROL_MODE];
  switch (command.control_mode) {
    case POLAR_VELOCITY_TARGET_MODE:
      PolarVelocityModeArgs_init(
        &command.mode_args.polar_velocity, &serialized->data[CONTROL_MODE_ARGS]);
      break;
  }
  command.target_global_pos[0] = convertTwoByteToFloat(
    serialized->data[TARGET_GLOBAL_POS_X_HIGH], serialized->data[TARGET_GLOBAL_POS_X_LOW], 32.767);
  command.target_global_pos[1] = convertTwoByteToFloat(
    serialized->data[TARGET_GLOBAL_POS_Y_HIGH], serialized->data[TARGET_GLOBAL_POS_Y_LOW], 32.767);
  command.terminal_velocity = convertTwoByteToFloat(
    serialized->data[TERMINAL_VELOCITY_HIGH], serialized->data[TERMINAL_VELOCITY_LOW], 32.767);
  return command;
}

// NOLINTEND

#endif  // CRANE_SENDER__ROBOT_PACKET_H_
