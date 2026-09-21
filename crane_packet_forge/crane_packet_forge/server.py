# Copyright (c) 2026 ibis-ssl
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""GUI のバックエンド。静的配信 + REST + WebSocket。

crane_web_debugger の WebSocket サーバ (8091) は ROS ノードで crane スタックの一部
なので使わない。このツールは crane が動いていなくても（むしろ動いていないほうが
良い状態で）バイト列を投げるのが目的なので、自前の小さなサーバを持つ。

送信ループは必ずここ（サーバ側）で回す。ブラウザから 1 フレームずつ送らせると、
WebSocket が一瞬詰まっただけで check_counter が 250ms 更新されず AI 断になる。
"""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
from typing import Any

from . import fields as F
from . import layout as L
from .assemble import assemble_command
from .decode import decode_command, hex_dump
from .events import EXIT_OK, EXIT_RUNTIME, EventLog
from .feedback import FeedbackWatcher
from .sender_loop import SenderOptions, SenderSession
from .spec import PacketSpec, SpecError
from .transport import find_conflicting_senders

PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_WEB_ROOT = PACKAGE_DIR.parent / "web"
PUSH_INTERVAL_S = 0.1  # GUI へ状態を押す間隔（送信そのものは 62.5Hz で別スレッド）
LOG_RING = 200


def resolve_shared_root(explicit: Path | None = None) -> Path | None:
    """crane_web_debugger の web/shared を探す。

    docker では compose が bind mount する。直接起動のときは ament の share か、
    ソースツリーの隣から拾う。見つからなければテーマ無しで動かす（致命的ではない）。
    """
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    env = os.environ.get("SHARED_ROOT")
    if env:
        candidates.append(Path(env))
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("crane_web_debugger")) / "web" / "shared"
        )
    except Exception:  # noqa: BLE001, S110 - docker や素の python では ament が無い。
        # 共有テーマが見つからなくても GUI 自体は動くので、ここは握りつぶしてよい。
        pass
    candidates.append(
        PACKAGE_DIR.parent.parent / "crane_web_debugger" / "web" / "shared"
    )

    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    return None


class ForgeState:
    """送信セッションとフィードバック購読を 1 つずつ持つ。

    同時に 2 本走らせない。2 つの送信元が同じ CM4 へ送ると check_counter が
    入り乱れるので、それをこのプロセス内で再現しないようにする。
    """

    def __init__(self, interface_ip: str | None = None) -> None:
        self.interface_ip = interface_ip
        self.spec = PacketSpec()
        self.session: SenderSession | None = None
        self.watcher: FeedbackWatcher | None = None
        self.logs: list[dict[str, Any]] = []
        self.log = EventLog(quiet=True, sink=self._record)

    def _record(self, record: dict[str, Any]) -> None:
        self.logs.append(record)
        del self.logs[:-LOG_RING]

    # --- 送信 ---

    def start(self, options: dict[str, Any]) -> None:
        self.stop("restart")
        session_options = SenderOptions(
            interface_ip=self.interface_ip,
            duration_s=options.get("duration_s"),
            freeze_counter=bool(options.get("freeze_counter", False)),
            stop_on_exit=bool(options.get("stop_on_exit", True)),
            abort_after_s=options.get("abort_after_s"),
            port=int(options.get("port", L.DEFAULT_PORT)),
        )
        self.session = SenderSession(self.spec, self.log, session_options)
        self.session.start()

    def stop(self, reason: str = "requested") -> None:
        if self.session is not None:
            self.session.stop(reason)
            self.session = None

    # --- フィードバック ---

    def watch(self, robot_id: int) -> None:
        self.unwatch()
        watcher = FeedbackWatcher(
            robot_id,
            interface_ip=self.interface_ip,
            on_event=lambda kind, message, extra: self.log.emit(kind, message, **extra),
        )
        watcher.start()
        self.watcher = watcher

    def unwatch(self) -> None:
        if self.watcher is not None:
            self.watcher.stop()
            self.watcher = None

    # --- 状態 ---

    def snapshot(self) -> dict[str, Any]:
        command = assemble_command(
            self.spec,
            check_counter=(
                self.session.stats.check_counter
                if self.session
                else self.spec.fields.get("check_counter", 0)
            ),
        )
        feedback = None
        if self.watcher is not None:
            stats = self.watcher.stats
            feedback = {
                "robot_id": self.watcher.robot_id,
                "group": self.watcher.group,
                "port": self.watcher.port,
                "received": stats.received,
                "bad_sync": stats.bad_sync,
                "rate": round(stats.rate(), 1),
                "silent": stats.silent_since is not None,
                "last": stats.last_feedback.to_json() if stats.last_feedback else None,
            }
        return {
            "spec": self.spec.to_json(),
            "address": self.spec.resolve_address(),
            "command_hex": command.hex(),
            "fields": [f.__dict__ for f in decode_command(command)],
            "warnings": self.spec.warnings(),
            "conflicts": find_conflicting_senders(),
            "sender": self.session.stats.to_json()
            if self.session
            else {"running": False},
            "feedback": feedback,
            "logs": self.logs[-40:],
        }


def resolve_web_root(explicit: Path | None = None) -> Path:
    """GUI の静的ファイルの場所。

    install 空間では Python モジュールは lib/pythonX/site-packages/ に、web は
    share/crane_packet_forge/web に入るので、モジュールの隣には無い。
    --symlink-install だと __file__ がソースツリーへ戻るため、この違いは
    開発中には現れない。素の install で壊れないよう share も見る。
    """
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    env = os.environ.get("WEB_ROOT")
    if env:
        candidates.append(Path(env))
    candidates.append(DEFAULT_WEB_ROOT)
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory("crane_packet_forge")) / "web"
        )
    except Exception:  # noqa: BLE001, S110 - ament が無い環境（docker / 素の python）
        pass

    for candidate in candidates:
        if (candidate / "index.html").is_file():
            return candidate
    raise FileNotFoundError(
        f"GUI の静的ファイルが見つからない（探した先: {[str(c) for c in candidates]}）。"
        "--web-root で指定してください。"
    )


def resolve_fonts_root(shared: Path | None) -> Path | None:
    """フォントの置き場を探す。空ディレクトリは「無い」と扱う。

    crane_web_debugger/web/fonts は docker のマウント点で、リポジトリ上は空。
    そのまま mount しても 404 になるだけなので、中身があるかまで見る。
    """
    candidates = [Path(os.environ["FONTS_DIR"])] if os.environ.get("FONTS_DIR") else []
    candidates.append(Path("/app/fonts"))
    if shared is not None:
        candidates.append(shared.parent / "fonts")

    for candidate in candidates:
        if candidate.is_dir() and any(candidate.iterdir()):
            return candidate
    return None


def create_app(
    *,
    interface_ip: str | None = None,
    web_root: Path | None = None,
    shared_root: Path | None = None,
):
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import JSONResponse
    from fastapi.staticfiles import StaticFiles

    state = ForgeState(interface_ip=interface_ip)
    app = FastAPI(title="Crane Packet Forge")
    app.state.forge = state

    # 【登録順】StaticFiles を "/" にマウントすると以降のルートが飲まれる。
    # API はすべてこれより前に登録する。crane_web_debugger の app.py と同じ罠。

    @app.get("/api/schema")
    def schema() -> JSONResponse:
        """GUI はフィールド表をハードコードせずここから取る。"""
        byte_owners = [F.owner_of_byte(i) for i in range(L.CMD_SIZE)]
        return JSONResponse(
            {
                "cmd_size": L.CMD_SIZE,
                "packet_size": L.PACKET_SIZE,
                "slots": L.SLOTS,
                "max_robot_id": L.MAX_ROBOT_ID,
                "check_counter_max": L.CHECK_COUNTER_MAX,
                "default_port": L.DEFAULT_PORT,
                "broadcast_address": L.BROADCAST_ADDRESS,
                "control_modes": L.CONTROL_MODES,
                "group_labels": F.GROUP_LABELS,
                "fields": [
                    {
                        "key": f.key,
                        "label": f.label,
                        "kind": f.kind,
                        "group": f.group,
                        "offset": f.offset,
                        "size": f.size,
                        "bit": f.bit,
                        "unit": f.unit,
                        "wire_range": f.wire_range,
                        "resolution": f.resolution,
                        "ui_min": f.ui_min,
                        "ui_max": f.ui_max,
                        "ui_step": f.ui_step,
                        "choices": [{"name": n, "value": v} for n, v in f.choices],
                        "mode": f.mode,
                        "note": f.note,
                        "bytes": list(f.byte_span),
                    }
                    for f in F.ALL_FIELDS
                ],
                "byte_owners": [owner.key if owner else None for owner in byte_owners],
                "byte_groups": [
                    owner.group if owner else None for owner in byte_owners
                ],
                "byte_descriptions": [F.describe_byte(i) for i in range(L.CMD_SIZE)],
            }
        )

    @app.get("/api/state")
    def get_state() -> JSONResponse:
        return JSONResponse(state.snapshot())

    @app.post("/api/spec")
    async def put_spec(payload: dict) -> JSONResponse:
        """GUI から spec をまるごと受け取る。CLI の --spec と同じ JSON。"""
        try:
            spec = PacketSpec.from_json(payload)
        except SpecError as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)
        state.spec = spec
        if state.session is not None:
            state.session.update_spec(spec)  # 送信を止めずに差し替える
        return JSONResponse(state.snapshot())

    @app.post("/api/preview")
    async def preview(payload: dict) -> JSONResponse:
        """送らずに組み立てだけする。CLI の --dry-run と同じ。"""
        try:
            spec = PacketSpec.from_json(payload)
        except SpecError as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)
        command = assemble_command(
            spec, check_counter=spec.fields.get("check_counter", 0)
        )
        return JSONResponse(
            {
                "command_hex": command.hex(),
                "hex_dump": hex_dump(command),
                "fields": [f.__dict__ for f in decode_command(command)],
                "warnings": spec.warnings(),
                "address": spec.resolve_address(),
            }
        )

    @app.post("/api/send/start")
    async def send_start(payload: dict | None = None) -> JSONResponse:
        try:
            state.start(payload or {})
        except OSError as exc:
            return JSONResponse(
                {"error": f"送信ソケットを開けない: {exc}"}, status_code=500
            )
        return JSONResponse(state.snapshot())

    @app.post("/api/send/stop")
    async def send_stop() -> JSONResponse:
        state.stop("requested")
        return JSONResponse(state.snapshot())

    @app.post("/api/watch/start")
    async def watch_start(payload: dict) -> JSONResponse:
        robot_id = int(payload.get("robot_id", state.spec.robot_id))
        try:
            state.watch(robot_id)
        except OSError as exc:
            return JSONResponse(
                {
                    "error": f"購読に失敗: {exc}。--interface-ip でロボット用 NIC を指定する"
                },
                status_code=500,
            )
        return JSONResponse(state.snapshot())

    @app.post("/api/watch/stop")
    async def watch_stop() -> JSONResponse:
        state.unwatch()
        return JSONResponse(state.snapshot())

    @app.websocket("/ws")
    async def websocket(ws: WebSocket) -> None:
        await ws.accept()
        try:
            while True:
                await ws.send_text(
                    json.dumps(state.snapshot(), ensure_ascii=False, default=str)
                )
                await asyncio.sleep(PUSH_INTERVAL_S)
        except WebSocketDisconnect:
            return
        except Exception:  # noqa: BLE001 - 1 本の接続の失敗でサーバを落とさない
            return

    shared = resolve_shared_root(shared_root)
    if shared is not None:
        app.mount("/shared", StaticFiles(directory=str(shared)), name="shared")

    fonts = resolve_fonts_root(shared)
    if fonts is not None:
        app.mount("/fonts", StaticFiles(directory=str(fonts)), name="fonts")
    else:
        # docker では Dockerfile が /app/fonts へ取得する。ホストで直接起動すると
        # 空なのでアイコンがリガチャの文字列で出る。黙って壊れるより言う。
        print(
            "フォントが見つかりません（アイコンが文字で表示されます）。"
            "FONTS_DIR を指定するか、crane_web_debugger/web/download_fonts.py で取得してください。"
        )

    root = resolve_web_root(web_root)
    app.mount("/", StaticFiles(directory=str(root), html=True), name="web-root")

    @app.on_event("shutdown")
    def _shutdown() -> None:
        state.stop("shutdown")  # 停止指令はここで飛ぶ
        state.unwatch()

    return app


def run_server(
    *,
    host: str = "0.0.0.0",
    port: int = 8094,
    interface_ip: str | None = None,
    web_root: Path | None = None,
    shared_root: Path | None = None,
) -> int:
    try:
        import uvicorn
    except ImportError:
        print("uvicorn / fastapi が要る: pip install fastapi 'uvicorn[standard]'")
        return EXIT_RUNTIME

    app = create_app(
        interface_ip=interface_ip, web_root=web_root, shared_root=shared_root
    )
    print(f"Crane Packet Forge: http://{host}:{port}/")
    uvicorn.run(app, host=host, port=port, log_level="warning")
    return EXIT_OK
