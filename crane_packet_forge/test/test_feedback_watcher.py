# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""FeedbackWatcher がイベントを EventLog へ流す経路を、ループバックの UDP で固定する。

multicast は環境に依存するので、購読ソケットだけをループバックの UDP に差し替える。
"""

import socket
import time
from typing import Any

import pytest
from crane_packet_forge.events import EventLog
from crane_packet_forge.feedback import FeedbackWatcher

from crane_packet_forge import feedback
from crane_packet_forge import layout as L


def _feedback_packet() -> bytes:
    data = bytearray(L.FEEDBACK_SIZE)
    data[feedback._OFF["SYNC_0"]], data[feedback._OFF["SYNC_1"]] = L.FEEDBACK_SYNC
    # 温度・電圧が全て 0 だと BOOT_LIKE になるので、温度を 1 つ入れておく
    data[feedback._OFF["TEMPERATURE_0"]] = 30
    return bytes(data)


def test_watch_start_and_rate_events_reach_event_log(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(0.1)
    monkeypatch.setattr(feedback, "open_multicast_socket", lambda *_a, **_k: sock)

    records: list[dict[str, Any]] = []
    log = EventLog(quiet=True, sink=records.append)
    watcher = FeedbackWatcher(
        3, on_event=lambda kind, message, extra: log.emit(kind, message, **extra)
    )
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        watcher.start()
        for _ in range(2):
            sender.sendto(_feedback_packet(), sock.getsockname())
            time.sleep(1.1)
    finally:
        watcher.stop()
        sender.close()

    kinds = [r["kind"] for r in records]
    assert kinds[0] == "WATCH_START"
    assert records[0]["robot_id"] == 3
    assert "RATE" in kinds
    rate = next(r for r in records if r["kind"] == "RATE")
    assert rate["robot_id"] == 3
    assert rate["bad_sync"] == 0
