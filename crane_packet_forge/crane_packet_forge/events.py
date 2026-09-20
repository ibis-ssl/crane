# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""イベント出力。人間向けの 1 行と、機械向けの JSONL を同じ場所から出す。

robot_bench は JSONL をファイル (--log) にしか出せず、stdout は常に人間向けだった。
エージェントから使うときに困るので、ここでは --json で stdout 自体を JSONL にする。
"""

from __future__ import annotations

import json
import sys
import threading
import time
from collections.abc import Callable
from datetime import datetime, timezone
from typing import Any, TextIO

# 終了コード。呼び出し側が失敗の種類で分岐できるようにする。
EXIT_OK = 0
EXIT_RUNTIME = 1
EXIT_USAGE = 2
EXIT_NETWORK = 3
EXIT_CONFLICT = 4
EXIT_INTERRUPTED = 130


def stamp(t: float) -> str:
    """ロボット側のログと突き合わせるのでローカル時刻で出す。"""
    local = datetime.fromtimestamp(t, tz=timezone.utc).astimezone()
    return local.strftime("%H:%M:%S.%f")[:-3]


class EventLog:
    """スレッド安全なイベント出力。

    json_stdout=True なら stdout は JSONL のみになる（人間向けの行は stderr へ）。
    """

    def __init__(
        self,
        path: str | None = None,
        *,
        json_stdout: bool = False,
        quiet: bool = False,
        sink: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self._lock = threading.Lock()
        # close() まで持ち続けるハンドルなので with は使えない
        self._fh: TextIO | None = (
            open(path, "a", encoding="utf-8") if path else None  # noqa: SIM115
        )
        self._json_stdout = json_stdout
        self._quiet = quiet
        self._sink = sink

    def emit(
        self, kind: str, message: str, *, t: float | None = None, **fields: Any
    ) -> None:
        now = time.time() if t is None else t
        record: dict[str, Any] = {
            "t": round(now, 3),
            "time": stamp(now),
            "kind": kind,
            "message": message,
        }
        record.update(fields)
        line = json.dumps(record, ensure_ascii=False)

        with self._lock:
            if self._json_stdout:
                print(line, file=sys.stdout, flush=True)
            elif not self._quiet:
                print(
                    f"{record['time']} {kind:<16} {message}",
                    file=sys.stdout,
                    flush=True,
                )
            if self._fh:
                self._fh.write(line + "\n")
                self._fh.flush()
        if self._sink is not None:
            self._sink(record)

    def warning(self, message: str, **fields: Any) -> None:
        """警告を出す。

        名前を `warn` にしない。ruff の自動修正が logging の非推奨 API と
        見なして `.warning(` へ書き換え、定義が無いメソッドの呼び出しになる。
        """
        self.emit("WARNING", message, **fields)

    def close(self) -> None:
        with self._lock:
            if self._fh:
                self._fh.close()
                self._fh = None
