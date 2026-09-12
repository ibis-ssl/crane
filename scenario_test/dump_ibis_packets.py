#!/usr/bin/env python3
"""crane が送出する ibis コマンドパケットをダンプして検証するツール。

`crane_sender/include/crane_sender/robot_packet.h` を正本としてデコードする。

想定する使い方（cm4_sim が無くても mode 4 の送出を確認できる）::

    # 端末1
    python3 scenario_test/dump_ibis_packets.py --port 12345
    # 端末2
    ros2 launch crane_bringup crane.launch.xml \
        sim:=true packet_type:=ibis planner:=visibility_graph

mode 4 は planner:=visibility_graph のときだけ出る。既定の rvo2 では mode 3 になる。

注意: このスクリプトは 12345 を占有する。simulator-cli や cm4_sim が同じポートを
listen していると取り合いになるので、ダンプ中はそれらを起動しないか、
simulator-cli 側を `--ibis-port 12346` へ退避させること。
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import socket
import sys
import time

# --- robot_packet.h のバイトレイアウト（正本） ---------------------------------
SLOT_SIZE = 65  # robot_id 1 byte + command 64 bytes
COMMAND_SIZE = 64
ROBOT_NUM = 11
PACKET_SIZE = SLOT_SIZE * ROBOT_NUM  # 715

HEADER = 0
CHECK_COUNTER = 1
VISION_GLOBAL_X_HIGH = 2
VISION_GLOBAL_Y_HIGH = 4
VISION_GLOBAL_THETA_HIGH = 6
TARGET_GLOBAL_THETA_HIGH = 8
KICK_POWER = 10
DRIBBLE_POWER = 11
ACCELERATION_LIMIT_HIGH = 12
LINEAR_VELOCITY_LIMIT_HIGH = 14
ANGULAR_VELOCITY_LIMIT_HIGH = 16
LATENCY_TIME_MS_HIGH = 18
ELAPSED_TIME_MS_HIGH = 20
FLAGS = 22
CONTROL_MODE = 23
CONTROL_MODE_ARGS = 24
TARGET_GLOBAL_POS_X_HIGH = 32
TARGET_GLOBAL_POS_Y_HIGH = 34
TERMINAL_VELOCITY_HIGH = 36

MODE_POLAR_VELOCITY_TARGET = 3
MODE_POSITION_TARGET_WITH_TERMINAL_VELOCITY = 4
MODE_NAMES = {
    MODE_POLAR_VELOCITY_TARGET: "POLAR_VELOCITY_TARGET_MODE(3)",
    MODE_POSITION_TARGET_WITH_TERMINAL_VELOCITY: "POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE(4)",
}

FLAG_IS_VISION_AVAILABLE = 0
FLAG_ENABLE_CHIP = 1
FLAG_STOP_EMERGENCY = 3


def decode_two_byte(data: bytes, offset: int, value_range: float) -> float:
    """convertTwoByteToFloat() と同じ復号。"""
    raw = (data[offset] << 8) | data[offset + 1]
    return (raw - 32767.0) / 32767.0 * value_range


def decode_uint16(data: bytes, offset: int) -> int:
    return (data[offset] << 8) | data[offset + 1]


def slot_is_empty(command: bytes) -> bool:
    """送信元が制御しないロボットのスロットはゼロ埋めされる。"""
    return not any(command)


def decode_command(command: bytes) -> dict:
    """64 バイトのコマンドを復号する。

    CONTROL_MODE_ARGS(24..31) は mode によって意味が変わる union なので、
    必ず CONTROL_MODE を先に見てから復号する。mode を見ずに読むと mode 4 の
    terminal_velocity_x/y が polar の r/theta として無言で解釈される。
    """
    mode = command[CONTROL_MODE]
    flags = command[FLAGS]

    decoded = {
        "header": command[HEADER],
        "check_counter": command[CHECK_COUNTER],
        "vision_global_pos": [
            decode_two_byte(command, VISION_GLOBAL_X_HIGH, 32.767),
            decode_two_byte(command, VISION_GLOBAL_Y_HIGH, 32.767),
        ],
        "vision_global_theta": decode_two_byte(
            command, VISION_GLOBAL_THETA_HIGH, math.pi
        ),
        "target_global_theta": decode_two_byte(
            command, TARGET_GLOBAL_THETA_HIGH, math.pi
        ),
        "kick_power": command[KICK_POWER] / 20.0,
        "dribble_power": command[DRIBBLE_POWER] / 20.0,
        "acceleration_limit": decode_two_byte(command, ACCELERATION_LIMIT_HIGH, 32.767),
        "linear_velocity_limit": decode_two_byte(
            command, LINEAR_VELOCITY_LIMIT_HIGH, 32.767
        ),
        "angular_velocity_limit": decode_two_byte(
            command, ANGULAR_VELOCITY_LIMIT_HIGH, 32.767
        ),
        "latency_time_ms": decode_uint16(command, LATENCY_TIME_MS_HIGH),
        "elapsed_time_ms_since_last_vision": decode_uint16(
            command, ELAPSED_TIME_MS_HIGH
        ),
        "is_vision_available": bool((flags >> FLAG_IS_VISION_AVAILABLE) & 1),
        "enable_chip": bool((flags >> FLAG_ENABLE_CHIP) & 1),
        "stop_emergency": bool((flags >> FLAG_STOP_EMERGENCY) & 1),
        "control_mode": mode,
        "control_mode_name": MODE_NAMES.get(mode, f"UNKNOWN({mode})"),
        # 32..37 は mode に依存しない固定フィールド
        "target_global_pos": [
            decode_two_byte(command, TARGET_GLOBAL_POS_X_HIGH, 32.767),
            decode_two_byte(command, TARGET_GLOBAL_POS_Y_HIGH, 32.767),
        ],
        "terminal_velocity": decode_two_byte(command, TERMINAL_VELOCITY_HIGH, 32.767),
    }

    if mode == MODE_POSITION_TARGET_WITH_TERMINAL_VELOCITY:
        decoded["mode_args"] = {
            "terminal_velocity_x": decode_two_byte(
                command, CONTROL_MODE_ARGS + 0, 32.767
            ),
            "terminal_velocity_y": decode_two_byte(
                command, CONTROL_MODE_ARGS + 2, 32.767
            ),
        }
    elif mode == MODE_POLAR_VELOCITY_TARGET:
        decoded["mode_args"] = {
            "target_global_velocity_r": decode_two_byte(
                command, CONTROL_MODE_ARGS + 0, 32.767
            ),
            "target_global_velocity_theta": decode_two_byte(
                command, CONTROL_MODE_ARGS + 2, 32.767
            ),
        }
    else:
        # 未知の mode の ARGS を復号してはならない（union なので意味が確定しない）
        decoded["mode_args"] = None

    return decoded


def format_slot(robot_id: int, decoded: dict) -> str:
    mode = decoded["control_mode"]
    mode_name = decoded["control_mode_name"]
    head = f"  robot {robot_id:2d} | byte23={mode} {mode_name}"
    args = decoded["mode_args"]
    if mode == MODE_POSITION_TARGET_WITH_TERMINAL_VELOCITY:
        tx, ty = decoded["target_global_pos"]
        tv = decoded["terminal_velocity"]
        vel_x = args["terminal_velocity_x"]
        vel_y = args["terminal_velocity_y"]
        body = (
            f"\n           target_pos=({tx:+.3f}, {ty:+.3f}) terminal_velocity={tv:.3f}"
            f"\n           args: terminal_velocity_xy=({vel_x:+.3f}, {vel_y:+.3f})"
        )
    elif mode == MODE_POLAR_VELOCITY_TARGET:
        vel_r = args["target_global_velocity_r"]
        vel_theta = args["target_global_velocity_theta"]
        body = f"\n           args: r={vel_r:+.3f} theta={vel_theta:+.3f}"
    else:
        body = "\n           (未知の mode。ARGS は復号しない)"
    vx, vy = decoded["vision_global_pos"]
    vtheta = decoded["vision_global_theta"]
    vision_ok = decoded["is_vision_available"]
    body += (
        f"\n           vision_pos=({vx:+.3f}, {vy:+.3f})"
        f" theta={vtheta:+.3f} vision_available={vision_ok}"
    )
    return head + body


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--port", type=int, default=12345, help="待ち受けポート (default: 12345)"
    )
    parser.add_argument(
        "--bind", default="0.0.0.0", help="待ち受けアドレス (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--count", type=int, default=0, help="受信するデータグラム数 (0: Ctrl-C まで)"
    )
    parser.add_argument(
        "--robot-id", type=int, default=None, help="このロボットIDのスロットだけ表示"
    )
    parser.add_argument(
        "--every",
        type=int,
        default=1,
        help="N データグラムに 1 回だけ表示 (default: 1)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="最初の受信までのタイムアウト秒 (default: 10)",
    )
    parser.add_argument("--json-out", default="", help="要約 JSON の出力先")
    parser.add_argument(
        "--expect-mode",
        type=int,
        default=MODE_POSITION_TARGET_WITH_TERMINAL_VELOCITY,
        choices=[
            MODE_POLAR_VELOCITY_TARGET,
            MODE_POSITION_TARGET_WITH_TERMINAL_VELOCITY,
        ],
        help="要約で期待する CONTROL_MODE。crane の送信ポート(12345)を見るなら 4、"
        "cm4_sim の出力ポート(12346)を見るなら 3 (default: 4)",
    )
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(args.timeout)

    print(
        f"listening on {args.bind}:{args.port} (expecting {PACKET_SIZE}-byte datagrams)"
    )
    print("Ctrl-C で終了し、要約を表示します\n")

    datagrams = 0
    size_mismatch = 0
    mode_counts = collections.defaultdict(
        collections.Counter
    )  # robot_id -> Counter(mode)
    last_decoded = {}
    started = time.time()

    try:
        while True:
            if args.count and datagrams >= args.count:
                break
            try:
                data, addr = sock.recvfrom(4096)
            except TimeoutError:
                print(
                    f"タイムアウト: {args.timeout:.1f} 秒間 1 つも受信できませんでした。",
                    file=sys.stderr,
                )
                print(
                    "crane が packet_type=ibis で起動しているか、送信先ポートが一致しているか確認してください。",
                    file=sys.stderr,
                )
                break
            sock.settimeout(None)
            datagrams += 1

            if len(data) != PACKET_SIZE:
                size_mismatch += 1
                print(
                    f"[warn] 想定外のサイズ {len(data)} バイト "
                    f"(期待 {PACKET_SIZE}) from {addr[0]}"
                )
                continue

            show = (datagrams % args.every) == 0
            if show:
                print(
                    f"=== datagram #{datagrams} from {addr[0]}:{addr[1]} "
                    f"({len(data)} bytes) ==="
                )

            for i in range(ROBOT_NUM):
                offset = i * SLOT_SIZE
                robot_id = data[offset]
                command = data[offset + 1 : offset + 1 + COMMAND_SIZE]

                if slot_is_empty(command):
                    continue
                if args.robot_id is not None and robot_id != args.robot_id:
                    continue

                decoded = decode_command(command)
                mode_counts[robot_id][decoded["control_mode"]] += 1
                last_decoded[robot_id] = decoded
                if show:
                    print(format_slot(robot_id, decoded))
            if show:
                print()
    except KeyboardInterrupt:
        print("\n(中断)")
    finally:
        sock.close()

    elapsed = time.time() - started
    print("\n===== 要約 =====")
    rate_hz = datagrams / elapsed if elapsed > 0 else 0.0
    print(f"受信データグラム: {datagrams} ({elapsed:.1f} 秒, {rate_hz:.1f} Hz)")
    if size_mismatch:
        print(f"サイズ不一致: {size_mismatch}")
    if not mode_counts:
        print("非空スロットが 1 つもありませんでした。")
    for robot_id in sorted(mode_counts):
        counts = mode_counts[robot_id]
        breakdown = ", ".join(
            f"{MODE_NAMES.get(m, f'UNKNOWN({m})')} x{c}"
            for m, c in sorted(counts.items())
        )
        print(f"  robot {robot_id:2d}: {breakdown}")

    all_modes = collections.Counter()
    for counts in mode_counts.values():
        all_modes.update(counts)
    if all_modes:
        expected = args.expect_mode
        if set(all_modes) == {expected}:
            print(f"\n=> 全スロットが byte23 == {expected} でした。")
        elif expected not in all_modes:
            if expected == MODE_POSITION_TARGET_WITH_TERMINAL_VELOCITY:
                hint = "planner:=visibility_graph で起動しているか確認してください。"
            else:
                hint = "cm4_sim が経路に入っているか確認してください。"
            print(f"\n=> byte23 == {expected} が 1 つも出ていません。{hint}")
        else:
            print(f"\n=> byte23 == {expected} と他モードが混在しています。")

    if args.json_out:
        summary = {
            "datagrams": datagrams,
            "elapsed_sec": elapsed,
            "size_mismatch": size_mismatch,
            "mode_counts": {
                str(r): {str(m): c for m, c in counts.items()}
                for r, counts in mode_counts.items()
            },
            "last_decoded": {str(r): d for r, d in last_decoded.items()},
        }
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        print(f"要約を {args.json_out} に書き出しました。")

    return 0


if __name__ == "__main__":
    sys.exit(main())
