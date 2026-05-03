"""SSL protobuf の自動コンパイルとモジュールロード."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

_CRANE_ROOT = Path(__file__).parent.parent.parent
_PROTO_SRC_DIR = _CRANE_ROOT / "consai_ros2" / "robocup_ssl_msgs" / "proto"
_SSL_PROTO_DIR = Path(__file__).parent.parent / "ssl_proto"

_REQUIRED_PROTOS = [
    "ssl_gc_common.proto",
    "ssl_gc_geometry.proto",
    "ssl_gc_game_event.proto",
    "ssl_gc_referee_message.proto",
    "ssl_vision_geometry.proto",
    "ssl_vision_detection.proto",
    "ssl_vision_detection_tracked.proto",
    "ssl_vision_wrapper.proto",
    "ssl_vision_wrapper_tracked.proto",
]

_SENTINEL = "ssl_gc_referee_message_pb2.py"


def _compile_protos() -> None:
    _SSL_PROTO_DIR.mkdir(parents=True, exist_ok=True)

    for fname in _REQUIRED_PROTOS:
        src = _PROTO_SRC_DIR / fname
        if not src.exists():
            raise RuntimeError(f"protoファイルが見つかりません: {src}")
        shutil.copy2(src, _SSL_PROTO_DIR / fname)

    proto_files = [str(_SSL_PROTO_DIR / f) for f in _REQUIRED_PROTOS]
    cmd_grpc = [
        sys.executable,
        "-m",
        "grpc_tools.protoc",
        f"--proto_path={_SSL_PROTO_DIR}",
        f"--python_out={_SSL_PROTO_DIR}",
        *proto_files,
    ]
    cmd_protoc = [
        "protoc",
        f"--proto_path={_SSL_PROTO_DIR}",
        f"--python_out={_SSL_PROTO_DIR}",
        *proto_files,
    ]

    for cmd in (cmd_grpc, cmd_protoc):
        try:
            subprocess.run(cmd, check=True, capture_output=True)
            (_SSL_PROTO_DIR / "__init__.py").touch()
            return
        except (subprocess.CalledProcessError, FileNotFoundError):
            continue

    raise RuntimeError(
        "protobufのコンパイルに失敗しました。以下のいずれかをインストールしてください:\n"
        "  pip install grpcio-tools\n"
        "  sudo apt install protobuf-compiler"
    )


def ensure_proto_compiled() -> None:
    if (_SSL_PROTO_DIR / _SENTINEL).exists():
        return
    print(f"[ssl_log] protobufをコンパイル中: {_SSL_PROTO_DIR}", flush=True)
    _compile_protos()
    print("[ssl_log] コンパイル完了", flush=True)


def load_proto_modules():
    """(SSL_WrapperPacket class, TrackerWrapperPacket class, Referee class) を返す."""
    ensure_proto_compiled()
    if str(_SSL_PROTO_DIR) not in sys.path:
        sys.path.insert(0, str(_SSL_PROTO_DIR))

    from ssl_vision_wrapper_pb2 import SSL_WrapperPacket  # noqa: PLC0415
    from ssl_vision_wrapper_tracked_pb2 import TrackerWrapperPacket  # noqa: PLC0415
    from ssl_gc_referee_message_pb2 import Referee  # noqa: PLC0415

    return SSL_WrapperPacket, TrackerWrapperPacket, Referee
