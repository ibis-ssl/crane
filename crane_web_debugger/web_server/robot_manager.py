"""Robot Manager: Raspberry Pi 上のロボットプロセスを操作するプロキシ。

もとは Orion_CM4 の host/robot-manager/server.py として別サービス・別イメージで
動いていた。テーマ（m3e-theme.css）を丸ごとコピーして持っていたためドリフトの
温床になっていたので、crane_web_debugger (8090) に内包する。Orion 側には
Pi 側 API の契約文書だけを残す。

Pi 側の契約（この 3 つだけに依存する）:
    GET  http://<ROBOT_IP_BASE><ROBOT_IP_OFFSET + id>:<ROBOT_PORT>/status
    POST 同 /start
    POST 同 /stop
status 以外に Pi が返すフィールド（voltage / temperatures / error_id /
error_info 等）はそのままパススルーする。

【sim で 192.168.20.0/24 に触らないこと】
以前は docker-dev.sh の `--scale robot-manager=0`（コンテナを起動しない）が
唯一の担保だった。内包でその仕組みが消えるので、環境変数 ROBOT_MANAGER_ENABLED
（既定 0）で代替する。false のときはネットワークに一切触れずに 503 を返すので、
sim ではパケットが 1 つも出ない（コンテナ不在よりも強い保証になる）。

【依存を増やさないこと】
エンドポイントを async def ではなく def で定義すると Starlette が自前の
スレッドプールで実行するので、標準ライブラリの urllib.request のまま移植できる。
13 台 × タイムアウト 0.8s のファンアウトに httpx の非同期性は要らない。
ただし Executor はモジュールレベルで別に持つ。Pi が全台無応答のときに
FastAPI 既定のスレッドプールを食い潰さないため。
"""

from __future__ import annotations

import json
import os
from concurrent.futures import ThreadPoolExecutor
from urllib.request import Request, urlopen

from fastapi import APIRouter, HTTPException

NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "13"))
ROBOT_IP_BASE = os.environ.get("ROBOT_IP_BASE", "192.168.20.")
ROBOT_IP_OFFSET = int(os.environ.get("ROBOT_IP_OFFSET", "100"))
ROBOT_PORT = int(os.environ.get("ROBOT_PORT", "8000"))
HTTP_TIMEOUT_SEC = float(os.environ.get("HTTP_TIMEOUT_SEC", "0.8"))


def is_enabled() -> bool:
    """sim では既定で無効。real 起動時だけ docker-dev.sh が 1 を立てる。"""
    return os.environ.get("ROBOT_MANAGER_ENABLED", "0").lower() in ("1", "true", "yes")


# Pi 全台が無応答でも FastAPI 本体のスレッドプールを塞がないよう専用に持つ
_executor = ThreadPoolExecutor(
    max_workers=min(8, NUM_ROBOTS), thread_name_prefix="robot-manager"
)

router = APIRouter(prefix="/api/robot-manager", tags=["robot-manager"])


def robot_ip(robot_id: int) -> str:
    return f"{ROBOT_IP_BASE}{ROBOT_IP_OFFSET + robot_id}"


def parse_status(success: bool, body_text: str) -> str:
    if not success:
        return "Offline"
    if not body_text:
        return "Running"
    try:
        body_json = json.loads(body_text)
    except json.JSONDecodeError:
        return "Running"
    status = body_json.get("status")
    if isinstance(status, str) and status:
        return status
    return "Running"


def send_pi_request(robot_id: int, method: str, path: str) -> tuple[bool, str]:
    target = f"http://{robot_ip(robot_id)}:{ROBOT_PORT}{path}"
    req = Request(target, method=method)
    try:
        with urlopen(req, timeout=HTTP_TIMEOUT_SEC) as resp:
            ok = 200 <= resp.status < 300
            return ok, resp.read().decode("utf-8", errors="replace")
    except Exception:  # noqa: BLE001 Pi 側の不調で API 全体を落とさない
        return False, ""


def _merge_passthrough(result: dict, ok: bool, body: str) -> dict:
    """Pi が返した追加フィールドを、こちらのキーを上書きしない形で足す。"""
    if not (ok and body):
        return result
    try:
        body_json = json.loads(body)
    except json.JSONDecodeError:
        return result
    for key, value in body_json.items():
        if key not in result:
            result[key] = value
    return result


def get_robot_status(robot_id: int) -> dict:
    ok, body = send_pi_request(robot_id, "GET", "/status")
    return _merge_passthrough(
        {
            "robot_id": robot_id,
            "ip": robot_ip(robot_id),
            "success": ok,
            "status": parse_status(ok, body),
        },
        ok,
        body,
    )


_COMMANDS = {
    "start": ("POST", "/start"),
    "stop": ("POST", "/stop"),
    "status": ("GET", "/status"),
}


def control_robot(robot_id: int, command: str) -> dict:
    method, path = _COMMANDS[command]
    ok, body = send_pi_request(robot_id, method, path)
    return _merge_passthrough(
        {
            "robot_id": robot_id,
            "command": command,
            "success": ok,
            "status": parse_status(ok, body),
        },
        ok,
        body,
    )


def _require_enabled() -> None:
    if not is_enabled():
        raise HTTPException(
            status_code=503,
            detail=(
                "Robot Manager は無効です。sim では 192.168.20.0/24 へ"
                " パケットを出さないよう既定で無効になっています。"
                " ./scripts/docker-dev.sh --robot-manager で有効化してください。"
            ),
        )


def _validate_id(robot_id: int) -> int:
    if not 0 <= robot_id < NUM_ROBOTS:
        raise HTTPException(status_code=400, detail="invalid robot_id")
    return robot_id


# config は無効時も 200 を返す。クライアントが「無効である」ことを
# 表示できるようにするため（ここで 503 にすると理由を出せない）
@router.get("/config")
def get_config() -> dict:
    return {
        "enabled": is_enabled(),
        "num_robots": NUM_ROBOTS,
        "ip_base": ROBOT_IP_BASE,
        "ip_offset": ROBOT_IP_OFFSET,
        "port": ROBOT_PORT,
        "timeout_sec": HTTP_TIMEOUT_SEC,
    }


@router.get("/robots")
def list_robots() -> dict:
    _require_enabled()
    robots = list(_executor.map(get_robot_status, range(NUM_ROBOTS)))
    return {"type": "pi_status", "robots": robots}


@router.get("/robots/{robot_id}/status")
def robot_status(robot_id: int) -> dict:
    _require_enabled()
    return control_robot(_validate_id(robot_id), "status")


@router.post("/robots/{robot_id}/start")
def robot_start(robot_id: int) -> dict:
    _require_enabled()
    return control_robot(_validate_id(robot_id), "start")


@router.post("/robots/{robot_id}/stop")
def robot_stop(robot_id: int) -> dict:
    _require_enabled()
    return control_robot(_validate_id(robot_id), "stop")
