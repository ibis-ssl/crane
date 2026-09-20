# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""robot_packet.h から自動生成したパケットレイアウト。手で編集しないこと。

再生成:
    python3 tools/gen_layout.py

生成元:
    crane_sender/include/crane_sender/robot_packet.h
    crane_sender/src/ibis_sender_node.cpp   (CommConfig の framing 定数)
    crane_robot_receiver/include/crane_robot_receiver/robot_feedback_protocol.hpp
    crane_robot_receiver/src/robot_receiver_node.cpp   (multicast の既定値)

test/test_layout_sync.py が生成をやり直してこのファイルと突き合わせるので、
ヘッダが動いたらテストが落ちる。
"""

import math

# --- framing (ibis_sender_node.cpp CommConfig) ---
CMD_SIZE = 64
SLOTS = 11
SLOT_SIZE = CMD_SIZE + 1
PACKET_SIZE = SLOT_SIZE * SLOTS
DEFAULT_PORT = 12345
BROADCAST_ADDRESS = "192.168.20.255"
MAX_ROBOT_ID = SLOTS - 1  # 11 スロット固定。11 番以上は符号化できない
CHECK_COUNTER_MAX = 200
MODE_ARGS_SIZE = 8

# --- robot_packet.h enum Address ---
HEADER = 0
CHECK_COUNTER = 1
VISION_GLOBAL_X_HIGH = 2
VISION_GLOBAL_X_LOW = 3
VISION_GLOBAL_Y_HIGH = 4
VISION_GLOBAL_Y_LOW = 5
VISION_GLOBAL_THETA_HIGH = 6
VISION_GLOBAL_THETA_LOW = 7
TARGET_GLOBAL_THETA_HIGH = 8
TARGET_GLOBAL_THETA_LOW = 9
KICK_POWER = 10
DRIBBLE_POWER = 11
ACCELERATION_LIMIT_HIGH = 12
ACCELERATION_LIMIT_LOW = 13
LINEAR_VELOCITY_LIMIT_HIGH = 14
LINEAR_VELOCITY_LIMIT_LOW = 15
ANGULAR_VELOCITY_LIMIT_HIGH = 16
ANGULAR_VELOCITY_LIMIT_LOW = 17
LATENCY_TIME_MS_HIGH = 18
LATENCY_TIME_MS_LOW = 19
ELAPSED_TIME_MS_SINCE_LAST_VISION_HIGH = 20
ELAPSED_TIME_MS_SINCE_LAST_VISION_LOW = 21
FLAGS = 22
CONTROL_MODE = 23
CONTROL_MODE_ARGS = 24
TARGET_GLOBAL_POS_X_HIGH = 32
TARGET_GLOBAL_POS_X_LOW = 33
TARGET_GLOBAL_POS_Y_HIGH = 34
TARGET_GLOBAL_POS_Y_LOW = 35
TERMINAL_VELOCITY_HIGH = 36
TERMINAL_VELOCITY_LOW = 37

# --- robot_packet.h enum FlagAddress ---
FLAG_BITS = {
    "IS_VISION_AVAILABLE": 0,
    "ENABLE_CHIP": 1,
    "STOP_EMERGENCY": 3,
}

# --- robot_packet.h ControlMode ---
CONTROL_MODES = {
    "POLAR_VELOCITY_TARGET_MODE": 3,
    "POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE": 4,
}

# --- serialize 本体の forward(..., range) から採取したレンジ ---
# 名前から推測してはいけない。mode 3 の theta は M_PI ではなく 32.767。
RANGES = {
    "ACCELERATION_LIMIT_HIGH": 32.767,
    "ANGULAR_VELOCITY_LIMIT_HIGH": 32.767,
    "LINEAR_VELOCITY_LIMIT_HIGH": 32.767,
    "POLAR:target_global_velocity_r": 32.767,
    "POLAR:target_global_velocity_theta": 32.767,
    "POSITION_TARGET:terminal_velocity_x": 32.767,
    "POSITION_TARGET:terminal_velocity_y": 32.767,
    "TARGET_GLOBAL_POS_X_HIGH": 32.767,
    "TARGET_GLOBAL_POS_Y_HIGH": 32.767,
    "TARGET_GLOBAL_THETA_HIGH": math.pi,
    "TERMINAL_VELOCITY_HIGH": 32.767,
    "VISION_GLOBAL_THETA_HIGH": math.pi,
    "VISION_GLOBAL_X_HIGH": 32.767,
    "VISION_GLOBAL_Y_HIGH": 32.767,
}

# --- mode_args union 内のオフセット (CONTROL_MODE_ARGS からの相対) ---
MODE_ARGS_OFFSETS = {
    "POLAR:target_global_velocity_r": 0,
    "POLAR:target_global_velocity_theta": 2,
    "POSITION_TARGET:terminal_velocity_x": 0,
    "POSITION_TARGET:terminal_velocity_y": 2,
}

# --- 128B フィードバック (robot_feedback_protocol.hpp) ---
FEEDBACK_SIZE = 128
FEEDBACK_SYNC = (0xAB, 0xEA)
FEEDBACK_TX_VALUE_COUNT = 14
MOTOR_CURRENT_SCALE = 10.0
KICK_STATE_SCALE = 10
FLOAT_SIZE = 4
# byte 2 は CRC ではない。G474 は定数 10 を書くだけなので検証に使わない
# (robot_feedback_protocol.hpp のコメントを参照)。判定は sync のみ。
FEEDBACK_OFFSETS = {
    "SYNC_0": 0,
    "SYNC_1": 1,
    "DUMMY_CRC": 2,
    "COUNTER": 3,
    "YAW_ANGLE": 4,
    "VOLTAGE_0": 8,
    "BALL_DETECTION_0": 12,
    "BALL_DETECTION_1": 13,
    "BALL_DETECTION_2": 14,
    "KICK_STATE": 15,
    "ERROR_ID": 16,
    "ERROR_INFO": 18,
    "ERROR_VALUE": 20,
    "MOTOR_CURRENT_0": 24,
    "MOTOR_CURRENT_1": 25,
    "MOTOR_CURRENT_2": 26,
    "MOTOR_CURRENT_3": 27,
    "BALL_DETECTION_3": 28,
    "TEMPERATURE_0": 29,
    "TEMPERATURE_1": 30,
    "TEMPERATURE_2": 31,
    "TEMPERATURE_3": 32,
    "TEMPERATURE_4": 33,
    "TEMPERATURE_5": 34,
    "TEMPERATURE_6": 35,
    "DIFF_ANGLE": 36,
    "VOLTAGE_1": 40,
    "ODOM_X": 44,
    "ODOM_Y": 48,
    "ODOM_SPEED_X": 52,
    "ODOM_SPEED_Y": 56,
    "CAMERA_POS_X_DIV2": 60,
    "CAMERA_POS_Y": 61,
    "CAMERA_RADIUS_DIV4": 62,
    "CAMERA_FPS": 63,
    "DEBUG_VALUES_START": 64,
    "MOUSE_ODOM_X": 64,
    "MOUSE_ODOM_Y": 68,
    "MOUSE_VEL_X": 72,
    "MOUSE_VEL_Y": 76,
    "RESERVED_START": 120,
}

# --- multicast の既定値 (robot_receiver_node.cpp) ---
MULTICAST_IP_BASE = "224.5.20"
FEEDBACK_PORT_BASE = 50100
IP_OCTET_OFFSET = 100
