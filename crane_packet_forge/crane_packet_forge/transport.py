# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""UDP 送信と、crane の送信ノードとの競合検出。"""

from __future__ import annotations

import os
import socket
import subprocess
from typing import Self

from . import layout as L


class Sender:
    """715 バイトのデータグラムを投げるだけのソケット。

    --interface-ip を渡したらそこへ bind する。複数 NIC の PC で送信元が
    ロボット用 LAN 以外に寄ると届かないので、解決したアドレスを外から読めるようにする。
    """

    def __init__(
        self,
        address: str,
        *,
        port: int = L.DEFAULT_PORT,
        interface_ip: str | None = None,
        broadcast: bool = False,
    ) -> None:
        self.address = address
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if broadcast:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        if interface_ip:
            self._sock.bind((interface_ip, 0))

    @property
    def source(self) -> str:
        host, port = self._sock.getsockname()
        return f"{host}:{port}"

    def send(self, datagram: bytes) -> None:
        self._sock.sendto(datagram, (self.address, self.port))

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


def find_conflicting_senders() -> list[str]:
    """crane の ibis_sender_node が動いていないか見る。

    拒否はしない（安全機構は緩和方針）。ただし 2 つの送信元が同じ CM4 へ送ると
    check_counter が入り乱れ、測定結果そのものが壊れるので必ず伝える。
    """
    try:
        out = subprocess.run(
            ["pgrep", "-af", "ibis_sender_node"],
            capture_output=True,
            text=True,
            timeout=5,
            # 見つからなければ rc=1。それは「競合なし」であって失敗ではない
            check=False,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    me = str(os.getpid())
    return [
        line
        for line in out.splitlines()
        if line.strip()
        and not line.startswith(me + " ")
        and "pgrep" not in line
        and "crane_packet_forge" not in line
        and "crane-forge" not in line
    ]
