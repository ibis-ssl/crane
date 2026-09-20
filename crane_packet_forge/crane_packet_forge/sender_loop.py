# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""62.5Hz の送信ループ。CLI と GUI サーバの両方がこれを使う。

ループをサーバ側に置くのは必須の設計。ブラウザから 1 フレームずつ送らせると、
WebSocket が一瞬詰まっただけで check_counter が 250ms 更新されず、CM4 は
AI 断とみなして止める。ブラウザは spec を更新するだけにして、送信はここが続ける。
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from dataclasses import field as dc_field
from typing import Any

from . import layout as L
from .assemble import assemble_command, build_datagram, stop_command
from .events import EventLog
from .spec import PacketSpec
from .transport import Sender

STOP_REPEAT = 5


def next_check_counter(counter: int) -> int:
    """crane と同じ巡回（`if (++counter_ > 200) counter_ = 0`）。"""
    return 0 if counter + 1 > L.CHECK_COUNTER_MAX else counter + 1


@dataclass
class LoopStats:
    sent: int = 0
    started_at: float | None = None
    last_sent_at: float | None = None
    check_counter: int = 0
    last_command: bytes = b""
    step_index: int | None = None
    running: bool = False
    stopped_reason: str = ""
    source: str = ""  # 解決した送信元 host:port。複数 NIC の切り分けで要る
    address: str = ""  # 実際の宛先

    def rate(self) -> float:
        if self.started_at is None or self.last_sent_at is None:
            return 0.0
        elapsed = self.last_sent_at - self.started_at
        return self.sent / elapsed if elapsed > 0 else 0.0

    def to_json(self) -> dict[str, Any]:
        return {
            "sent": self.sent,
            "rate": round(self.rate(), 2),
            "check_counter": self.check_counter,
            "step_index": self.step_index,
            "running": self.running,
            "stopped_reason": self.stopped_reason,
            "source": self.source,
            "address": self.address,
            "last_command_hex": self.last_command.hex(),
        }


@dataclass
class SenderOptions:
    interface_ip: str | None = None
    duration_s: float | None = None
    freeze_counter: bool = False
    stop_on_exit: bool = True
    abort_after_s: float | None = None
    port: int = L.DEFAULT_PORT
    on_tick: Callable[[LoopStats], None] | None = dc_field(default=None)


class SenderSession:
    """1 本の送信ループ。start() で背景スレッドが回り、stop() で止まる。

    実行中に update_spec() で spec を差し替えられる（GUI のスライダー操作用）。
    check_counter はここが持つので、spec を差し替えてもカウンタは途切れない。
    """

    def __init__(
        self, spec: PacketSpec, log: EventLog, options: SenderOptions | None = None
    ) -> None:
        self._spec = spec
        self._log = log
        self._opt = options or SenderOptions()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sender: Sender | None = None
        self.stats = LoopStats()

    # --- 制御 ---

    @property
    def spec(self) -> PacketSpec:
        with self._lock:
            return self._spec

    def update_spec(self, spec: PacketSpec) -> None:
        """送信を止めずに中身を差し替える。宛先の変更は次回 start() から効く。"""
        with self._lock:
            self._spec = spec

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("すでに送信中")
        spec = self.spec
        address = spec.resolve_address()
        self._sender = Sender(
            address,
            port=self._opt.port,
            interface_ip=self._opt.interface_ip,
            broadcast=spec.broadcast,
        )
        self._stop.clear()
        self.stats = LoopStats(
            running=True,
            started_at=time.time(),
            source=self._sender.source,
            address=f"{address}:{self._opt.port}",
        )
        self._log.emit(
            "SEND_START",
            f"{address}:{self._opt.port} へ {spec.rate_hz}Hz で送信（送信元 {self._sender.source}）",
            address=address,
            port=self._opt.port,
            source=self._sender.source,
            rate_hz=spec.rate_hz,
            robot_id=spec.robot_id,
        )
        self._thread = threading.Thread(
            target=self._run, name="forge-sender", daemon=True
        )
        self._thread.start()

    def stop(self, reason: str = "requested") -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5.0)
            self._thread = None
        self.stats.running = False
        self.stats.stopped_reason = reason

    def wait(self, timeout: float | None = None) -> None:
        if self._thread:
            self._thread.join(timeout=timeout)

    # --- ループ本体 ---

    def _run(self) -> None:
        assert self._sender is not None
        spec = self.spec
        interval = 1.0 / spec.rate_hz
        started = time.time()
        next_tick = started
        counter = 0
        aborted = False

        try:
            while not self._stop.is_set():
                now = time.time()
                spec = self.spec

                if (
                    self._opt.abort_after_s is not None
                    and now - started >= self._opt.abort_after_s
                ):
                    # crane が停止指令を出さずに落ちた状況の再現。finally を通さない。
                    aborted = True
                    self._log.emit(
                        "SEND_ABORT",
                        f"{self._opt.abort_after_s}s 経過。停止指令を送らずに打ち切る",
                        elapsed_s=round(now - started, 3),
                    )
                    break

                if (
                    self._opt.duration_s is not None
                    and now - started >= self._opt.duration_s
                ):
                    self.stats.stopped_reason = "duration"
                    break

                overrides, step_index, step_done = self._current_step(
                    spec, now - started
                )
                if step_done:
                    self.stats.stopped_reason = "steps"
                    break
                if step_index != self.stats.step_index:
                    self.stats.step_index = step_index
                    if step_index is not None:
                        self._log.emit(
                            "STEP",
                            f"step {step_index}: {overrides}",
                            step_index=step_index,
                            fields=dict(overrides),
                        )

                sent_counter = (
                    spec.fields.get("check_counter", 0)
                    if self._opt.freeze_counter
                    else counter
                )
                command = assemble_command(
                    spec, check_counter=sent_counter, overrides=overrides
                )
                self._sender.send(build_datagram(command, spec.robot_id))

                self.stats.sent += 1
                self.stats.last_sent_at = time.time()
                self.stats.check_counter = sent_counter
                self.stats.last_command = command
                if self._opt.on_tick is not None:
                    self._opt.on_tick(self.stats)

                if not self._opt.freeze_counter:
                    counter = next_check_counter(counter)

                next_tick += interval
                sleep_for = next_tick - time.time()
                if sleep_for > 0:
                    self._stop.wait(sleep_for)
                else:
                    next_tick = time.time()  # 遅れたら取り返さずに仕切り直す
        finally:
            # abort は「停止指令を送らずに落ちる」再現なので、停止指令だけ飛ばす。
            # ソケットは閉じる（ファイルディスクリプタを残さない）。
            self._finish(counter, send_stop=not aborted)

    def _finish(self, counter: int, *, send_stop: bool) -> None:
        """終了の後始末。停止指令を送り、集計を出してソケットを閉じる。"""
        if send_stop and self._opt.stop_on_exit:
            self._send_stop(counter)
        self.stats.running = False
        if not send_stop:
            self.stats.stopped_reason = "abort"
        self._log.emit(
            "SEND_END",
            f"{self.stats.sent} パケット送信（実測 {self.stats.rate():.1f}Hz）",
            sent=self.stats.sent,
            rate=round(self.stats.rate(), 2),
            reason=self.stats.stopped_reason or "requested",
        )
        if self._sender:
            self._sender.close()
            self._sender = None

    def _send_stop(self, counter: int) -> None:
        """終了時に速度 0 + STOP_EMERGENCY を繰り返す。落ちても止まるように。

        counter は「次に送る値」なので、先に進めない。進めると本体の最後の値と
        停止指令の間で 1 つ飛び、ログを突き合わせたときに欠落に見える。
        """
        if self._sender is None:
            return
        spec = self.spec
        for _ in range(STOP_REPEAT):
            command = stop_command(spec, check_counter=counter)
            try:
                self._sender.send(build_datagram(command, spec.robot_id))
            except OSError:
                break
            counter = next_check_counter(counter)
            time.sleep(1.0 / max(spec.rate_hz, 1.0))
        self._log.emit(
            "SEND_STOP",
            f"停止指令 (r=0 + STOP_EMERGENCY) を {STOP_REPEAT} 回送った",
            repeat=STOP_REPEAT,
        )

    def _current_step(
        self, spec: PacketSpec, elapsed: float
    ) -> tuple[dict[str, Any], int | None, bool]:
        """steps のうち今どこにいるか。(上書き, index, 全部終わったか)"""
        if not spec.steps:
            return {}, None, False
        acc = 0.0
        for index, step in enumerate(spec.steps):
            acc += step.duration_s
            if elapsed < acc:
                return dict(step.fields), index, False
        return {}, None, True
