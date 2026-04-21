"""HTTP server for crane_web_debugger."""

from __future__ import annotations

import argparse
import os
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


# uvicorn の "app:app" import-string 起動 (--reload) に対応するためのモジュールレベル変数。
# 環境変数 WEB_ROOT でルートディレクトリを指定できる（デフォルト: /app/web）。
app = create_app(Path(os.environ.get("WEB_ROOT", "/app/web")))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="crane web debugger HTTP server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--web-root", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    uvicorn.run(
        create_app(args.web_root), host=args.host, port=args.port, log_level="warning"
    )


if __name__ == "__main__":
    main()
