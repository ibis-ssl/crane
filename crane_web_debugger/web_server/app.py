"""HTTP server for crane_web_debugger."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import uvicorn
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

try:  # uvicorn の --app-dir /app 起動とパッケージ起動の両方に対応する
    from robot_manager import router as robot_manager_router
except ImportError:  # pragma: no cover
    from .robot_manager import router as robot_manager_router


def create_app(web_root: Path) -> FastAPI:
    """アプリを組み立てる。

    【登録順は変えないこと】StaticFiles を "/" にマウントすると、Starlette は
    登録順にマッチするので、それより後に足したルートは全部静的配信に飲まれる。
    API ルーターは必ず最後のマウントより前に登録する。このファイルは短いので
    追記すると自然に末尾へ書いてしまう。そこが罠になる。
    """
    app = FastAPI(title="Crane Web Debugger HTTP")

    # 1. API ルーター（"/" マウントより前）
    app.include_router(robot_manager_router)

    # 2. フォント
    fonts_dir = Path(os.environ.get("FONTS_DIR", "/app/fonts"))
    if fonts_dir.is_dir():
        app.mount("/fonts", StaticFiles(directory=str(fonts_dir)), name="fonts")

    # 3. 静的配信（これ以降にルートを足しても効かない）
    app.mount(
        "/",
        StaticFiles(directory=str(web_root), html=True, follow_symlink=True),
        name="web-root",
    )
    return app


# uvicorn の "app:app" import-string 起動 (--reload) に対応するためのモジュールレベル変数。
# 環境変数 WEB_ROOT でルートディレクトリを指定できる（デフォルト: /app/web）。
app = create_app(Path(os.environ.get("WEB_ROOT", "/app/web")))


def main() -> None:
    parser = argparse.ArgumentParser(description="crane web debugger HTTP server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--web-root", type=Path, required=True)
    args = parser.parse_args()
    uvicorn.run(
        create_app(args.web_root), host=args.host, port=args.port, log_level="warning"
    )


if __name__ == "__main__":
    main()
