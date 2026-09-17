#!/usr/bin/env python3
"""合成したロボット feedback パケットを crane_robot_receiver へ注入するツール。

128 バイトのパケットレイアウトの正本は実機ファームウェア
(G474_Orion_main の `Core/Src/ai_comm.c` sendRobotInfo()) であり、
`crane_robot_receiver/include/crane_robot_receiver/robot_feedback_protocol.hpp`
はその crane 側の対応表である。
注入するパケットは crane の受信側ではなく実機の送信側に合わせること。

cm4_sim がまだ無くても、crane 側の feedback 受信経路だけを検証できる。

新構成（CM4 in the loop）の検証::

    # 端末1: crane を起動する（sim:=true でも multicast 受信で起動する）
    ros2 launch crane_bringup crane.launch.xml sim:=true
    # 端末2
    python3 scenario_test/inject_feedback.py --robot-id 0
    # 端末3
    ros2 topic echo /robot_feedback --once

従来構成（unicast 受け、feedback_sim_mode:=true）を試す場合は --unicast を付ける。

crane_robot_receiver は受信から 100ms より古いデータを配列に入れないので、
--rate は既定の 125Hz のまま流し続けること。
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time

PACKET_SIZE = 128

# --- パケットオフセット（正本は G474 の ai_comm.c sendRobotInfo()） -----------
SYNC_0 = 0
SYNC_1 = 1
# byte 2 はチェックサムではない。実機ファームウェア (G474_Orion_main の
# Core/Src/ai_comm.c sendRobotInfo()) は `buf[2] = 10;  // CRC, 10:dummy` と
# 定数を書くだけなので、この注入ツールも実機と同じ定数を書く。
# ここに本物のチェックサムを書くと、実機では成立しない前提が sim でだけ成立し、
# 「受信側で byte 2 を検証する」誤った修正が sim で緑になってしまう。
DUMMY_CRC = 2
DUMMY_CRC_VALUE = 10
COUNTER = 3
YAW_ANGLE = 4
VOLTAGE_0 = 8
BALL_DETECTION_0 = 12
KICK_STATE = 15
ERROR_ID = 16
MOTOR_CURRENT_0 = 24
BALL_DETECTION_3 = 28
TEMPERATURE_0 = 29
DIFF_ANGLE = 36
VOLTAGE_1 = 40
ODOM_X = 44
ODOM_Y = 48
ODOM_SPEED_X = 52
ODOM_SPEED_Y = 56
CAMERA_POS_X_DIV2 = 60
CAMERA_FPS = 63
MOUSE_ODOM_X = 64
MOUSE_ODOM_Y = 68
MOUSE_VEL_X = 72
MOUSE_VEL_Y = 76

SYNC_0_VALUE = 0xAB
SYNC_1_VALUE = 0xEA


def put_float(buf: bytearray, offset: int, value: float) -> None:
    """受信側は memcpy で読むのでネイティブ（リトルエンディアン）の float32。"""
    struct.pack_into("<f", buf, offset, value)


def build_packet(
    counter: int, x: float, y: float, yaw: float, vx: float, vy: float
) -> bytes:
    buf = bytearray(PACKET_SIZE)
    buf[SYNC_0] = SYNC_0_VALUE
    buf[SYNC_1] = SYNC_1_VALUE
    buf[DUMMY_CRC] = DUMMY_CRC_VALUE
    buf[COUNTER] = counter & 0xFF

    put_float(buf, YAW_ANGLE, yaw)
    put_float(buf, VOLTAGE_0, 24.0)
    put_float(buf, VOLTAGE_1, 24.0)
    put_float(buf, DIFF_ANGLE, 0.0)

    put_float(buf, ODOM_X, x)
    put_float(buf, ODOM_Y, y)
    put_float(buf, ODOM_SPEED_X, vx)
    put_float(buf, ODOM_SPEED_Y, vy)

    # mouse odom は DEBUG_VALUES_START と領域を共有している（既知の設計上の重なり）
    put_float(buf, MOUSE_ODOM_X, x)
    put_float(buf, MOUSE_ODOM_Y, y)
    put_float(buf, MOUSE_VEL_X, vx)
    put_float(buf, MOUSE_VEL_Y, vy)

    buf[BALL_DETECTION_0] = 0
    buf[BALL_DETECTION_3] = 0
    buf[KICK_STATE] = 0
    # error_id = 0（エラーなし）
    buf[ERROR_ID] = 0
    for i in range(4):
        buf[MOTOR_CURRENT_0 + i] = 5  # 0.5A 相当（scale 10）
    for i in range(7):
        buf[TEMPERATURE_0 + i] = 30
    # ローカルカメラは未接続扱いでゼロ埋め（実機のカメラ未接続時と同じ）
    for i in range(CAMERA_POS_X_DIV2, CAMERA_FPS + 1):
        buf[i] = 0

    return bytes(buf)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--robot-id",
        type=int,
        action="append",
        default=None,
        help="対象ロボットID（複数指定可）。既定は 0",
    )
    parser.add_argument(
        "--multicast-base",
        default="224.5.20",
        help="multicast アドレスのベース (default: 224.5.20)",
    )
    parser.add_argument(
        "--ip-offset",
        type=int,
        default=100,
        help="最終オクテットのオフセット (default: 100)",
    )
    parser.add_argument(
        "--port-base", type=int, default=50100, help="ポートのベース (default: 50100)"
    )
    parser.add_argument(
        "--unicast",
        action="store_true",
        help="multicast ではなく 127.0.0.1 へ unicast する（feedback_sim_mode:=true 用）",
    )
    parser.add_argument(
        "--rate", type=float, default=125.0, help="送信レート Hz (default: 125)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="送信時間 秒 (0: Ctrl-C まで, default: 10)",
    )
    parser.add_argument("--ttl", type=int, default=1, help="multicast TTL (default: 1)")
    parser.add_argument(
        "--multicast-if",
        default="127.0.0.1",
        help="multicast 送出インタフェースの IP (default: 127.0.0.1)。"
        "既定ではループバックに固定し Wi-Fi/LAN への漏洩を防ぐ。"
        "実機ネットワークへ流したい場合はそのインタフェースの IP を指定する",
    )
    parser.add_argument(
        "--moving",
        action="store_true",
        help="位置を円運動させる（静止値ではなく変化を見たいとき）",
    )
    args = parser.parse_args()

    robot_ids = args.robot_id if args.robot_id else [0]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if not args.unicast:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, args.ttl)
        # 同一ホストの受信プロセスへ届かせる
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        # 送出インタフェースを明示する。指定しないと OS の既定ルート（Wi-Fi に
        # なりうる）が選ばれ、224.5.20.x が AP へ漏れる。ルーティング設定に
        # 依存せずソケット単位で固定するため、発生源での封じ込めになる。
        sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_MULTICAST_IF,
            socket.inet_aton(args.multicast_if),
        )

    targets = []
    for rid in robot_ids:
        if args.unicast:
            addr = "127.0.0.1"
        else:
            addr = f"{args.multicast_base}.{rid + args.ip_offset}"
        targets.append((rid, addr, args.port_base + rid))

    cast_kind = "unicast" if args.unicast else "multicast"
    for rid, addr, port in targets:
        print(f"robot {rid} -> {addr}:{port} ({cast_kind})")
    duration_text = "infinite" if args.duration == 0 else f"{args.duration:.1f} s"
    print(f"rate={args.rate:.1f} Hz duration={duration_text}")

    period = 1.0 / args.rate if args.rate > 0 else 0.008
    started = time.time()
    counter = 0
    sent = 0

    try:
        while True:
            now = time.time()
            elapsed = now - started
            if args.duration and elapsed >= args.duration:
                break

            if args.moving:
                x = 2.0 * math.cos(elapsed)
                y = 2.0 * math.sin(elapsed)
                vx = -2.0 * math.sin(elapsed)
                vy = 2.0 * math.cos(elapsed)
                yaw = math.atan2(vy, vx)
            else:
                x, y, vx, vy, yaw = -1.0, 0.5, 0.0, 0.0, 0.0

            packet = build_packet(counter, x, y, yaw, vx, vy)
            for _rid, addr, port in targets:
                sock.sendto(packet, (addr, port))
                sent += 1
            counter += 1

            sleep_for = period - (time.time() - now)
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("\n(中断)")
    finally:
        sock.close()

    print(f"送信パケット数: {sent} ({time.time() - started:.1f} 秒)")
    print("crane 側で `ros2 topic echo /robot_feedback --once` を確認してください。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
