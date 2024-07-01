// Copyright (c) 2023 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef CRANE_SENDER__ROBOT_PACKET_H_
#define CRANE_SENDER__ROBOT_PACKET_H_
#include <cmath>
#include <utility>

// NOLINTBEGIN(readability/casting)

struct TwoByte
{
  uint8_t high;
  uint8_t low;
};

TwoByte convertFloatToTwoByte(float val, float range)
{
  uint16_t uint16 = (uint16_t)(32767.f * (float)(val / range) + 32767.f);
  return {(uint16 & 0xFF00) >> 8, uint16 & 0x00FF};
}

float convertTwoByteToFloat(uint8_t byte_high, uint8_t byte_low, float range)
{
  uint16_t two_byte = (byte_high << 8) | byte_low;
  return (float)(two_byte - 32767.f) / 32767.f * range;
}

void forward(uint8_t * arg1, uint8_t * arg2, float val, float range)
{
  auto two_byte = convertFloatToTwoByte(val, range);
  *arg1 = two_byte.high;
  *arg2 = two_byte.low;
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

#define FLOAT_FROM_2BYTE(name, range)  \
  packet.name = convertTwoByteToFloat( \
    data[(int)(Address::name##_HIGH)], data[(int)(Address::name##_LOW)], range)

#define FLOAT_FROM_1BYTE(name, range) packet.name = data[(int)(Address::name)] / 255.0f * range

    //    packet.HEADER = data[(int)(Address::HEADER)];
    packet.CHECK = data[(int)(Address::CHECK)];
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
    const uint8_t kick_raw = data[(int)(Address::KICK_POWER)];
    if (kick_raw >= 101) {
      packet.CHIP_ENABLE = true;
      packet.KICK_POWER = (kick_raw - 101) / 20.0f;
    } else {
      packet.CHIP_ENABLE = false;
      packet.KICK_POWER = kick_raw / 20.0f;
    }
    packet.DRIBBLE_POWER = data[(int)(Address::DRIBBLE_POWER)] / 20.0f;

    uint8_t local_flags = data[(int)(Address::LOCAL_FLAGS)];
    packet.IS_ID_VISIBLE = local_flags & 0x01;
    packet.LOCAL_KEEPER_MODE_ENABLE = local_flags & 0x02;
    packet.STOP_FLAG = local_flags & 0x04;
    packet.LOCAL_FEEDBACK_ENABLE = local_flags & 0x08;
    packet.IS_DRIBBLER_UP = local_flags & 0x10;

#undef FLOAT_FROM_1BYTE
#undef FLOAT_FROM_2BYTE

    return packet;
  }

  uint8_t data[(int)(Address::SIZE)];
};

RobotCommand::operator RobotCommandSerialized() const
{
  RobotCommandSerialized serialized;

#define FLOAT_TO_2BYTE(name, range)                                                             \
  std::pair<uint8_t, uint8_t> name##_two_byte = convertFloatToTwoByte(name, range);             \
  serialized.data[(int)(RobotCommandSerialized::Address::name##_HIGH)] = name##_two_byte.first; \
  serialized.data[(int)(RobotCommandSerialized::Address::name##_LOW)] = name##_two_byte.second

#define FLOAT_TO_1BYTE(name, range)                           \
  uint8_t name##_one_byte = (uint8_t)(name / range * 255.0f); \
  serialized.data[(int)(RobotCommandSerialized::Address::name)] = name##_one_byte

  //  serialized.data[(int)(RobotCommandSerialized::Address::HEADER)] = HEADER;
  serialized.data[(int)(RobotCommandSerialized::Address::CHECK)] = CHECK;

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
  serialized.data[(int)(RobotCommandSerialized::Address::DRIBBLE_POWER)] =
    (uint8_t)(DRIBBLE_POWER * 20);
  serialized.data[(int)(RobotCommandSerialized::Address::KICK_POWER)] = [&]() -> uint8_t {
    if (CHIP_ENABLE) {
      return (uint8_t)((std::round(20 * KICK_POWER) + 101));
    } else {
      return (uint8_t)(std::round(20 * KICK_POWER));
    }
  }();

  uint8_t local_flags = 0x00;
  local_flags |= (IS_ID_VISIBLE << 0);
  local_flags |= (LOCAL_KEEPER_MODE_ENABLE << 1);
  local_flags |= (STOP_FLAG << 2);
  local_flags |= (LOCAL_FEEDBACK_ENABLE << 3);
  local_flags |= (IS_DRIBBLER_UP << 4);

  serialized.data[(int)(RobotCommandSerialized::Address::LOCAL_FLAGS)] = local_flags;

#undef FLOAT_TO_1BYTE
#undef FLOAT_TO_2BYTE

  return serialized;
}

#define MODE_ARGS_SIZE (8)

struct LocalCameraModeArgs
{
  explicit LocalCameraModeArgs(uint8_t * args)
  {
    ball_x = convertTwoByteToFloat(args[0], args[1], 32.767);
    ball_y = convertTwoByteToFloat(args[2], args[3], 32.767);
    ball_vx = convertTwoByteToFloat(args[4], args[5], 32.767);
    ball_vy = convertTwoByteToFloat(args[6], args[7], 32.767);
    target_global_vel_x = convertTwoByteToFloat(args[8], args[9], 32.767);
    target_global_vel_y = convertTwoByteToFloat(args[10], args[11], 32.767);
  }

  void serialize(uint8_t * args)
  {
    forward(&args[0], &args[1], ball_x, 32.767);
    forward(&args[2], &args[3], ball_y, 32.767);
    forward(&args[4], &args[5], ball_vx, 32.767);
    forward(&args[6], &args[7], ball_vy, 32.767);
    forward(&args[8], &args[9], target_global_vel_x, 32.767);
    forward(&args[10], &args[11], target_global_vel_y, 32.767);
  }

  // ボール情報
  float ball_x;
  float ball_y;
  float ball_vx;
  float ball_vy;

  // ボールが見えない場合のフォールバック用
  float target_global_vel_x;
  float target_global_vel_y;
};

struct PositionTargetModeArgs
{
  explicit PositionTargetModeArgs(uint8_t * args)
  {
    target_global_x = convertTwoByteToFloat(args[0], args[1], 32.767);
    target_global_y = convertTwoByteToFloat(args[2], args[3], 32.767);
    speed_limit_at_target = convertTwoByteToFloat(args[4], args[5], 32.767);
  }

  void serialize(uint8_t * args)
  {
    forward(&args[0], &args[1], target_global_x, 32.767);
    forward(&args[2], &args[3], target_global_y, 32.767);
    forward(&args[4], &args[5], speed_limit_at_target, 32.767);
  }

  // 目標位置
  float target_global_x;
  float target_global_y;
  // 到達時の速度制限
  float speed_limit_at_target;
};

struct SimpleVelocityTargetModeArgs
{
  explicit SimpleVelocityTargetModeArgs(uint8_t * args)
  {
    target_global_vx = convertTwoByteToFloat(args[0], args[1], 32.767);
    target_global_vy = convertTwoByteToFloat(args[2], args[3], 32.767);
  }

  void serialize(uint8_t * args)
  {
    forward(&args[0], &args[1], target_global_vx, 32.767);
    forward(&args[2], &args[3], target_global_vy, 32.767);
  }

  // 目標速度
  float target_global_vx;
  float target_global_vy;
};

struct VelocityTargetWithTrajectoryModeArgs
{
  explicit VelocityTargetWithTrajectoryModeArgs(uint8_t * args)
  {
    target_global_vx = convertTwoByteToFloat(args[0], args[1], 32.767);
    target_global_vy = convertTwoByteToFloat(args[2], args[3], 32.767);
    trajectory_global_origin_x = convertTwoByteToFloat(args[4], args[5], 32.767);
    trajectory_global_origin_y = convertTwoByteToFloat(args[6], args[7], 32.767);
    trajectory_origin_angle = convertTwoByteToFloat(args[8], args[9], M_PI);
    trajectory_curvature = convertTwoByteToFloat(args[10], args[11], 32.767);
  }

  void serialize(uint8_t * args)
  {
    forward(&args[0], &args[1], target_global_vx, 32.767);
    forward(&args[2], &args[3], target_global_vy, 32.767);
    forward(&args[4], &args[5], trajectory_global_origin_x, 32.767);
    forward(&args[6], &args[7], trajectory_global_origin_y, 32.767);
    forward(&args[8], &args[9], trajectory_origin_angle, M_PI);
    forward(&args[10], &args[11], trajectory_curvature, 32.767);
  }

  // 目標速度
  float target_global_vx;
  float target_global_vy;

  // ベース座標
  float trajectory_global_origin_x;
  float trajectory_global_origin_y;

  // 軌道の初期角度
  float trajectory_origin_angle;
  // 軌道の曲率
  float trajectory_curvature;
};

struct RobotCommandSerializedV2;

struct RobotCommandV2
{
  uint8_t header;
  uint8_t check_counter;

  // 共通情報
  //  ビジョン
  float vision_global_x;
  float vision_global_y;
  float vision_global_theta;
  bool is_vision_available;
  //  指令
  float target_global_theta;
  float kick_power;
  float dribble_power;
  bool enable_chip;
  bool lift_dribbler;
  bool stop_emergency;
  //  制御補助情報
  float speed_limit;
  float omega_limit;
  bool prioritize_move;                   // falseなら回転を優先
  bool prioritize_accurate_acceleration;  // falseなら異方性を考慮せずに加速
  // 制御モードと自由記述欄
  enum ControlMode {
    LOCAL_CAMERA_MODE = 0,
    POSITION_TARGET_MODE = 1,
    SIMPLE_VELOCITY_TARGET_MODE = 2,
    VELOCITY_TARGET_WITH_TRAJECTORY_MODE = 3,
  } control_mode;

  LocalCameraModeArgs * local_camera_mode_args;
  PositionTargetModeArgs * position_target_mode_args;
  SimpleVelocityTargetModeArgs * simple_velocity_target_mode_args;
  VelocityTargetWithTrajectoryModeArgs * velocity_target_with_trajectory_mode_args;

  operator RobotCommandSerializedV2() const;
};

struct RobotCommandSerializedV2
{
  enum Address {
    HEADER,
    CHECK_COUNTER,
    VISION_GLOBAL_X_HIGH,
    VISION_GLOBAL_X_LOW,
    VISION_GLOBAL_Y_HIGH,
    VISION_GLOBAL_Y_LOW,
    VISION_GLOBAL_THETA_HIGH,
    VISION_GLOBAL_THETA_LOW,
    //    IS_VISION_AVAILABLE,
    TARGET_GLOBAL_THETA_HIGH,
    TARGET_GLOBAL_THETA_LOW,
    KICK_POWER,
    DRIBBLE_POWER,
    //    ENABLE_CHIP,
    //    LIFT_DRIBBLER,
    //    STOP_EMERGENCY,
    SPEED_LIMIT_HIGH,
    SPEED_LIMIT_LOW,
    OMEGA_LIMIT_HIGH,
    OMEGA_LIMIT_LOW,
    //    PRIORITIZE_MOVE,
    //    PRIORITIZE_ACCURATE_ACCELERATION,
    FLAGS,
    CONTROL_MODE,
    CONTROL_MODE_ARGS,
    //    POSITION_TARGET_MODE_ARGS,
    //    SIMPLE_VELOCITY_TARGET_MODE_ARGS,
    //    VELOCITY_TARGET_WITH_TRAJECTORY_MODE_ARGS,
  };

  enum FlagAddress {
    IS_VISION_AVAILABLE = 0,
    ENABLE_CHIP = 1,
    LIFT_DRIBBLER = 2,
    STOP_EMERGENCY = 3,
    PRIORITIZE_MOVE = 4,
    PRIORITIZE_ACCURATE_ACCELERATION = 5,
  };

  explicit RobotCommandSerializedV2(const RobotCommandV2 command)
  {
    data[Address::HEADER] = command.header;
    data[Address::CHECK_COUNTER] = command.check_counter;
    forward(
      &data[Address::VISION_GLOBAL_X_HIGH], &data[Address::VISION_GLOBAL_X_LOW],
      command.vision_global_x, 32.767);
    forward(
      &data[Address::VISION_GLOBAL_Y_HIGH], &data[Address::VISION_GLOBAL_Y_LOW],
      command.vision_global_y, 32.767);
    forward(
      &data[Address::VISION_GLOBAL_THETA_HIGH], &data[Address::VISION_GLOBAL_THETA_LOW],
      command.vision_global_theta, M_PI);
    forward(
      &data[Address::TARGET_GLOBAL_THETA_HIGH], &data[Address::TARGET_GLOBAL_THETA_LOW],
      command.target_global_theta, M_PI);
    data[Address::KICK_POWER] = command.kick_power * 20;
    data[Address::DRIBBLE_POWER] = command.dribble_power * 20;
    forward(
      &data[Address::SPEED_LIMIT_HIGH], &data[Address::SPEED_LIMIT_LOW], command.speed_limit,
      32.767);
    forward(
      &data[Address::OMEGA_LIMIT_HIGH], &data[Address::OMEGA_LIMIT_LOW], command.omega_limit,
      32.767);
    uint8_t flags = 0x00;
    flags |= (command.is_vision_available << FlagAddress::IS_VISION_AVAILABLE);
    flags |= (command.enable_chip << FlagAddress::ENABLE_CHIP);
    flags |= (command.lift_dribbler << FlagAddress::LIFT_DRIBBLER);
    flags |= (command.stop_emergency << FlagAddress::STOP_EMERGENCY);
    flags |= (command.prioritize_move << FlagAddress::PRIORITIZE_MOVE);
    flags |=
      (command.prioritize_accurate_acceleration << FlagAddress::PRIORITIZE_ACCURATE_ACCELERATION);
    data[Address::FLAGS] = flags;
    data[Address::CONTROL_MODE] = (uint8_t)command.control_mode;
    switch (command.control_mode) {
      case RobotCommandV2::ControlMode::LOCAL_CAMERA_MODE:
        command.local_camera_mode_args->serialize(&data[Address::CONTROL_MODE_ARGS]);
        break;
      case RobotCommandV2::ControlMode::POSITION_TARGET_MODE:
        command.position_target_mode_args->serialize(&data[Address::CONTROL_MODE_ARGS]);
        break;
      case RobotCommandV2::ControlMode::SIMPLE_VELOCITY_TARGET_MODE:
        command.simple_velocity_target_mode_args->serialize(&data[Address::CONTROL_MODE_ARGS]);
        break;
      case RobotCommandV2::ControlMode::VELOCITY_TARGET_WITH_TRAJECTORY_MODE:
        command.velocity_target_with_trajectory_mode_args->serialize(
          &data[Address::CONTROL_MODE_ARGS]);
        break;
    }
  }

  uint8_t data[64];
};

// NOLINTEND(readability/casting)

#endif  // CRANE_SENDER__ROBOT_PACKET_H_
