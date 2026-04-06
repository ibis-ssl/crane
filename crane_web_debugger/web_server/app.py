"""HTTP server for crane_web_debugger."""

from __future__ import annotations

import argparse
from pathlib import Path

import uvicorn
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles


def create_app(web_root: Path) -> FastAPI:
    app = FastAPI(title="Crane Web Debugger HTTP")

    app.mount(
        "/",
        StaticFiles(directory=str(web_root), html=True, follow_symlink=True),
        name="web-root",
    )
    return app


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="crane web debugger HTTP server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--web-root", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    app = create_app(args.web_root)
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
