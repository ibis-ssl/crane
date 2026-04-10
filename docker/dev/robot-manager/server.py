#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import signal
import threading
from concurrent.futures import ThreadPoolExecutor
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError
from urllib.request import Request, urlopen

NUM_ROBOTS = int(os.environ.get("NUM_ROBOTS", "13"))
ROBOT_IP_BASE = os.environ.get("ROBOT_IP_BASE", "192.168.20.")
ROBOT_IP_OFFSET = int(os.environ.get("ROBOT_IP_OFFSET", "100"))
ROBOT_PORT = int(os.environ.get("ROBOT_PORT", "8000"))
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8090"))
HTTP_TIMEOUT_SEC = float(os.environ.get("HTTP_TIMEOUT_SEC", "0.8"))

WEB_DIR = Path(__file__).resolve().parent / "web"


def robot_ip(robot_id: int) -> str:
    return f"{ROBOT_IP_BASE}{ROBOT_IP_OFFSET + robot_id}"


def parse_status(success: bool, body_text: str, default_ok: str = "Running") -> str:
    if not success:
        return "Offline"
    if not body_text:
        return default_ok
    try:
        body_json = json.loads(body_text)
    except json.JSONDecodeError:
        return default_ok
    status = body_json.get("status")
    if isinstance(status, str) and status:
        return status
    return default_ok


def send_pi_request(robot_id: int, method: str, path: str) -> tuple[bool, str]:
    target = f"http://{robot_ip(robot_id)}:{ROBOT_PORT}{path}"
    req = Request(target, method=method)
    try:
        with urlopen(req, timeout=HTTP_TIMEOUT_SEC) as resp:
            ok = 200 <= resp.status < 300
            body = resp.read().decode("utf-8", errors="replace")
            return ok, body
    except URLError:
        return False, ""
    except TimeoutError:
        return False, ""
    except Exception:
        return False, ""


def get_robot_status(robot_id: int) -> dict:
    ok, body = send_pi_request(robot_id, "GET", "/status")
    result: dict = {
        "robot_id": robot_id,
        "ip": robot_ip(robot_id),
        "success": ok,
        "status": parse_status(ok, body),
    }
    # Pi が返す追加フィールド（voltage, temperatures, error_id, error_info 等）をパススルー
    if ok and body:
        try:
            body_json = json.loads(body)
            for key, value in body_json.items():
                if key not in result:
                    result[key] = value
        except json.JSONDecodeError:
            pass
    return result


def control_robot(robot_id: int, command: str) -> dict:
    command_map = {
        "start": ("POST", "/start"),
        "stop": ("POST", "/stop"),
        "status": ("GET", "/status"),
    }
    if command not in command_map:
        return {
            "robot_id": robot_id,
            "command": command,
            "success": False,
            "status": "Unknown command",
        }
    method, path = command_map[command]
    ok, body = send_pi_request(robot_id, method, path)
    result: dict = {
        "robot_id": robot_id,
        "command": command,
        "success": ok,
        "status": parse_status(ok, body),
    }
    # Pi が返す追加フィールドをパススルー
    if ok and body:
        try:
            body_json = json.loads(body)
            for key, value in body_json.items():
                if key not in result:
                    result[key] = value
        except json.JSONDecodeError:
            pass
    return result


class Handler(BaseHTTPRequestHandler):
    def _json(self, payload: dict | list, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _text(self, text: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_static(self, rel_path: str) -> None:
        rel = rel_path.lstrip("/")
        if rel == "":
            rel = "index.html"
        target = (WEB_DIR / rel).resolve()
        if not str(target).startswith(str(WEB_DIR.resolve())) or not target.exists():
            self._text("Not Found", HTTPStatus.NOT_FOUND)
            return

        content_type = "application/octet-stream"
        if target.suffix == ".html":
            content_type = "text/html; charset=utf-8"
        elif target.suffix == ".js":
            content_type = "application/javascript; charset=utf-8"
        elif target.suffix == ".css":
            content_type = "text/css; charset=utf-8"
        elif target.suffix == ".svg":
            content_type = "image/svg+xml"
        elif target.suffix == ".woff2":
            content_type = "font/woff2"
        elif target.suffix == ".woff":
            content_type = "font/woff"
        elif target.suffix == ".ttf":
            content_type = "font/ttf"
        elif target.suffix == ".ico":
            content_type = "image/x-icon"

        data = target.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _parse_robot_id(self, text: str) -> int | None:
        try:
            robot_id = int(text)
        except ValueError:
            return None
        if 0 <= robot_id < NUM_ROBOTS:
            return robot_id
        return None

    def do_GET(self) -> None:
        if self.path == "/healthz":
            self._json({"ok": True, "service": "robot-manager"})
            return

        if self.path == "/api/robots":
            with ThreadPoolExecutor(max_workers=min(8, NUM_ROBOTS)) as ex:
                robots = list(ex.map(get_robot_status, range(NUM_ROBOTS)))
            self._json({"type": "pi_status", "robots": robots})
            return

        if self.path.startswith("/api/robots/") and self.path.endswith("/status"):
            robot_id_str = self.path[len("/api/robots/") : -len("/status")]
            robot_id = self._parse_robot_id(robot_id_str.strip("/"))
            if robot_id is None:
                self._json({"error": "invalid robot_id"}, HTTPStatus.BAD_REQUEST)
                return
            self._json(control_robot(robot_id, "status"))
            return

        self._serve_static(self.path)

    def do_POST(self) -> None:
        if self.path.startswith("/api/robots/") and self.path.endswith("/start"):
            robot_id_str = self.path[len("/api/robots/") : -len("/start")]
            robot_id = self._parse_robot_id(robot_id_str.strip("/"))
            if robot_id is None:
                self._json({"error": "invalid robot_id"}, HTTPStatus.BAD_REQUEST)
                return
            self._json(control_robot(robot_id, "start"))
            return

        if self.path.startswith("/api/robots/") and self.path.endswith("/stop"):
            robot_id_str = self.path[len("/api/robots/") : -len("/stop")]
            robot_id = self._parse_robot_id(robot_id_str.strip("/"))
            if robot_id is None:
                self._json({"error": "invalid robot_id"}, HTTPStatus.BAD_REQUEST)
                return
            self._json(control_robot(robot_id, "stop"))
            return

        self._json({"error": "Not Found"}, HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return


def main() -> None:
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"robot-manager listening on http://0.0.0.0:{HTTP_PORT}")

    def handle_signal(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    server.serve_forever()


if __name__ == "__main__":
    main()
