# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""送信ループを実際のソケットで回す。

check_counter の連続性と終了時の停止指令は、組み立て単体では確かめられない
（ループが持つ状態なので）。ループバックへ実際に送って受け取って見る。
"""

import socket
import threading
import time
from itertools import pairwise
from typing import Self

import pytest
from crane_packet_forge.decode import split_datagram
from crane_packet_forge.events import EventLog
from crane_packet_forge.sender_loop import (
    STOP_REPEAT,
    SenderOptions,
    SenderSession,
    next_check_counter,
)
from crane_packet_forge.spec import PacketSpec

from crane_packet_forge import layout as L

STOP_BIT = 1 << L.FLAG_BITS["STOP_EMERGENCY"]


class Capture:
    """ループバックの 12345 を受けて、対象スロットの 64B だけ貯める。"""

    def __init__(self, robot_id: int) -> None:
        self.robot_id = robot_id
        self.packets: list[bytes] = []
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(0.3)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    @property
    def port(self) -> int:
        return self._sock.getsockname()[1]

    def __enter__(self) -> Self:
        self._thread.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        self._sock.close()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(4096)
            except TimeoutError:
                continue
            except OSError:
                break
            if len(data) == L.PACKET_SIZE:
                self.packets.append(split_datagram(data)[self.robot_id])


def run_session(
    robot_id: int, fields: dict[str, object], *, stop_on_exit: bool = True
) -> list[bytes]:
    with Capture(robot_id) as capture:
        spec = PacketSpec(target="127.0.0.1", robot_id=robot_id, rate_hz=50.0)
        spec.fields.update(fields)
        session = SenderSession(
            spec,
            EventLog(quiet=True),
            SenderOptions(
                interface_ip="127.0.0.1",
                duration_s=0.4,
                port=capture.port,
                stop_on_exit=stop_on_exit,
            ),
        )
        session.start()
        session.wait(timeout=5)
        time.sleep(0.5)  # 停止指令が届くまで待つ
        return list(capture.packets)


def test_check_counter_is_contiguous_including_the_stop_sequence() -> None:
    """本体と停止指令の間でカウンタが飛ばない。

    受信側は「変化していること」しか見ないので動作には響かないが、
    ログを突き合わせたときに欠落に見える。
    """
    packets = run_session(3, {"control_mode": 3, "flags.is_vision_available": True})
    assert len(packets) > STOP_REPEAT

    counters = [p[L.CHECK_COUNTER] for p in packets]
    for previous, current in pairwise(counters):
        assert current == next_check_counter(previous), (
            f"check_counter が {previous} → {current} で飛んでいる"
        )


def test_stop_sequence_is_sent_on_exit() -> None:
    packets = run_session(
        4,
        {
            "control_mode": 3,
            "polar.target_global_velocity_r": 0.4,
            "kick_power": 0.5,
            "flags.is_vision_available": True,
        },
    )
    tail = packets[-STOP_REPEAT:]
    assert len(tail) == STOP_REPEAT
    for packet in tail:
        assert packet[L.FLAGS] & STOP_BIT, "STOP_EMERGENCY が立っていない"
        assert packet[L.KICK_POWER] == 0
        # 速度 0 は 0x7fff（生ゼロではない）
        assert packet[L.CONTROL_MODE_ARGS : L.CONTROL_MODE_ARGS + 2] == b"\x7f\xff"

    # 本体側では停止フラグが立っていないこと（常時 STOP では試験にならない）
    assert not any(p[L.FLAGS] & STOP_BIT for p in packets[:-STOP_REPEAT])


def test_no_stop_on_exit_leaves_the_robot_commanded() -> None:
    """--no-stop-on-exit / --abort-after の経路。停止指令を送らない。"""
    packets = run_session(
        2,
        {
            "control_mode": 3,
            "polar.target_global_velocity_r": 0.4,
            "flags.is_vision_available": True,
        },
        stop_on_exit=False,
    )
    assert packets
    assert not any(p[L.FLAGS] & STOP_BIT for p in packets)


@pytest.mark.parametrize("value", [0, 1, L.CHECK_COUNTER_MAX - 1, L.CHECK_COUNTER_MAX])
def test_counter_wraps_like_crane(value) -> None:
    expected = 0 if value == L.CHECK_COUNTER_MAX else value + 1
    assert next_check_counter(value) == expected
