# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""128 バイトのロボットフィードバックを multicast で受ける。

crane_robot_receiver と同じ 224.5.20.(100+id):(50100+id) を購読する。
unicast の 127.0.0.1:50100+id を bind してはいけない。SO_REUSEPORT の 4-tuple
ハッシュで cm4-sim とパケットを奪い合い、「位置制御が効かない」ように見える
(docs/network.md)。multicast なら再配信を横から見るだけで済む。
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from dataclasses import field as dc_field

from . import layout as L

_OFF = L.FEEDBACK_OFFSETS


def multicast_endpoint(robot_id: int) -> tuple[str, int]:
    return (
        f"{L.MULTICAST_IP_BASE}.{robot_id + L.IP_OCTET_OFFSET}",
        L.FEEDBACK_PORT_BASE + robot_id,
    )


def open_multicast_socket(
    group: str, port: int, interface_ip: str | None = None
) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    iface = (
        socket.inet_aton(interface_ip)
        if interface_ip
        else struct.pack("=I", socket.INADDR_ANY)
    )
    sock.setsockopt(
        socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(group) + iface
    )
    sock.settimeout(0.5)
    return sock


def _f32(data: bytes, offset: int) -> float:
    return struct.unpack_from("<f", data, offset)[0]


def _u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


@dataclass
class Feedback:
    """crane_robot_receiver の RobotFeedback と同じ読み方をした 128B。"""

    counter: int
    yaw_angle: float
    diff_angle: float
    voltage: tuple[float, float]
    temperature: tuple[int, ...]
    motor_current: tuple[float, ...]
    ball_detection: tuple[int, ...]
    kick_state: int
    error_id: int
    error_info: int
    error_value: float
    odom: tuple[float, float]
    odom_speed: tuple[float, float]
    mouse_odom: tuple[float, float]
    mouse_vel: tuple[float, float]
    camera: tuple[int, int, int, int]
    values: tuple[float, ...]
    sync_valid: bool
    boot_like: bool

    def to_json(self) -> dict[str, object]:
        return {
            "counter": self.counter,
            "yaw_angle": self.yaw_angle,
            "diff_angle": self.diff_angle,
            "voltage": list(self.voltage),
            "temperature": list(self.temperature),
            "motor_current": list(self.motor_current),
            "ball_detection": list(self.ball_detection),
            "kick_state": self.kick_state,
            "error_id": self.error_id,
            "error_info": self.error_info,
            "error_value": self.error_value,
            "odom": list(self.odom),
            "odom_speed": list(self.odom_speed),
            "mouse_odom": list(self.mouse_odom),
            "mouse_vel": list(self.mouse_vel),
            "camera": list(self.camera),
            "values": list(self.values),
            "sync_valid": self.sync_valid,
            "boot_like": self.boot_like,
        }


def decode_feedback(data: bytes) -> Feedback:
    if len(data) != L.FEEDBACK_SIZE:
        raise ValueError(f"フィードバックは {L.FEEDBACK_SIZE} バイト: {len(data)}")

    sync_valid = (data[_OFF["SYNC_0"]], data[_OFF["SYNC_1"]]) == L.FEEDBACK_SYNC
    temperature = tuple(data[_OFF[f"TEMPERATURE_{i}"]] for i in range(7))
    voltage = (_f32(data, _OFF["VOLTAGE_0"]), _f32(data, _OFF["VOLTAGE_1"]))
    values = tuple(
        _f32(data, _OFF["DEBUG_VALUES_START"] + i * L.FLOAT_SIZE)
        for i in range(L.FEEDBACK_TX_VALUE_COUNT)
    )

    return Feedback(
        counter=data[_OFF["COUNTER"]],
        yaw_angle=_f32(data, _OFF["YAW_ANGLE"]),
        diff_angle=_f32(data, _OFF["DIFF_ANGLE"]),
        voltage=voltage,
        temperature=temperature,
        motor_current=tuple(
            data[_OFF[f"MOTOR_CURRENT_{i}"]] / L.MOTOR_CURRENT_SCALE for i in range(4)
        ),
        ball_detection=tuple(data[_OFF[f"BALL_DETECTION_{i}"]] for i in range(4)),
        kick_state=data[_OFF["KICK_STATE"]] * L.KICK_STATE_SCALE,
        error_id=_u16(data, _OFF["ERROR_ID"]),
        error_info=_u16(data, _OFF["ERROR_INFO"]),
        error_value=_f32(data, _OFF["ERROR_VALUE"]),
        odom=(_f32(data, _OFF["ODOM_X"]), _f32(data, _OFF["ODOM_Y"])),
        odom_speed=(_f32(data, _OFF["ODOM_SPEED_X"]), _f32(data, _OFF["ODOM_SPEED_Y"])),
        mouse_odom=(_f32(data, _OFF["MOUSE_ODOM_X"]), _f32(data, _OFF["MOUSE_ODOM_Y"])),
        mouse_vel=(_f32(data, _OFF["MOUSE_VEL_X"]), _f32(data, _OFF["MOUSE_VEL_Y"])),
        camera=(
            data[_OFF["CAMERA_POS_X_DIV2"]],
            data[_OFF["CAMERA_POS_Y"]],
            data[_OFF["CAMERA_RADIUS_DIV4"]],
            data[_OFF["CAMERA_FPS"]],
        ),
        values=values,
        sync_valid=sync_valid,
        # 温度と電圧が全て 0 のパケットは G474 のリセット直後の値
        boot_like=all(t == 0 for t in temperature)
        and voltage[0] == 0.0
        and voltage[1] == 0.0,
    )


@dataclass
class WatchStats:
    received: int = 0
    bad_sync: int = 0
    last_rx: float | None = None
    last_feedback: Feedback | None = None
    silent_since: float | None = None
    _second_count: int = 0
    _second_bad: int = 0
    _second_start: float = dc_field(default_factory=time.time)

    def rate(self) -> float:
        elapsed = time.time() - self._second_start
        return self._second_count / elapsed if elapsed > 0 else 0.0


class FeedbackWatcher:
    """1 機体ぶんのフィードバックを購読する背景スレッド。

    無音・復帰・リセット直後を検出して on_event へ渡す。crane_robot_receiver が
    同じ group を購読していても multicast なので競合しない。
    """

    SILENCE_S = 1.5

    def __init__(
        self,
        robot_id: int,
        *,
        interface_ip: str | None = None,
        on_event: Callable[[str, str, dict], None] | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.group, self.port = multicast_endpoint(robot_id)
        self._interface_ip = interface_ip
        self._on_event = on_event or (lambda kind, message, fields: None)
        self.stats = WatchStats()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None

    def start(self) -> None:
        self._sock = open_multicast_socket(self.group, self.port, self._interface_ip)
        self._thread = threading.Thread(
            target=self._run, name=f"feedback-{self.robot_id}", daemon=True
        )
        self._thread.start()
        self._on_event(
            "WATCH_START",
            f"robot {self.robot_id}: {self.group}:{self.port} を購読",
            {"robot_id": self.robot_id, "group": self.group, "port": self.port},
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._sock:
            self._sock.close()
            self._sock = None

    def _run(self) -> None:
        assert self._sock is not None
        last_rate_report = time.time()
        while not self._stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(2048)
            except TimeoutError:
                self._check_silence()
                continue
            except OSError:
                break

            now = time.time()
            if len(data) != L.FEEDBACK_SIZE:
                self.stats.bad_sync += 1
                continue

            feedback = decode_feedback(data)
            if not feedback.sync_valid:
                # byte 2 は CRC ではないので検証に使わない。同期バイトだけを見る。
                self.stats.bad_sync += 1
                self.stats._second_bad += 1
                continue

            if self.stats.silent_since is not None:
                self._on_event(
                    "RESUME",
                    f"robot {self.robot_id}: {now - self.stats.silent_since:.2f}s の無音から復帰",
                    {
                        "robot_id": self.robot_id,
                        "silence_s": round(now - self.stats.silent_since, 3),
                    },
                )
                self.stats.silent_since = None
            if feedback.boot_like:
                self._on_event(
                    "BOOT_LIKE",
                    f"robot {self.robot_id}: 温度・電圧が全て 0（G474 リセット直後）",
                    {"robot_id": self.robot_id},
                )

            self.stats.received += 1
            self.stats._second_count += 1
            self.stats.last_rx = now
            self.stats.last_feedback = feedback

            if now - last_rate_report >= 1.0:
                self._on_event(
                    "RATE",
                    f"robot {self.robot_id}: {self.stats.rate():.0f} pkt/s bad_sync={self.stats._second_bad}",
                    {
                        "robot_id": self.robot_id,
                        "rate": round(self.stats.rate(), 1),
                        "bad_sync": self.stats._second_bad,
                        "counter": feedback.counter,
                        "voltage": list(feedback.voltage),
                    },
                )
                last_rate_report = now
                self.stats._second_count = 0
                self.stats._second_bad = 0
                self.stats._second_start = now

    def _check_silence(self) -> None:
        now = time.time()
        if self.stats.last_rx is None:
            return
        if (
            self.stats.silent_since is None
            and now - self.stats.last_rx > self.SILENCE_S
        ):
            self.stats.silent_since = self.stats.last_rx
            self._on_event(
                "SILENCE",
                f"robot {self.robot_id}: {self.SILENCE_S}s 以上パケットが来ない",
                {"robot_id": self.robot_id, "last_rx": round(self.stats.last_rx, 3)},
            )
