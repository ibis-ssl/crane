"""MCAP読み込み共通ユーティリティ."""

import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


def resolve_mcap_path(path: Path) -> Path:
    """ディレクトリから.mcapファイルを解決する."""
    if path.is_dir():
        mcap_files = list(path.glob("*.mcap"))
        if not mcap_files:
            raise FileNotFoundError(f"No .mcap file found in {path}")
        if len(mcap_files) > 1:
            logger.warning(f"Multiple .mcap files found, using {mcap_files[0]}")
        return mcap_files[0]
    return path


def open_sequential_reader(mcap_path: Path):
    """SequentialReaderを初期化して返す."""
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    except ImportError as e:
        raise ImportError(
            "rosbag2_py が必要です。ROS 2 rosbag2パッケージをインストールしてください。"
        ) from e

    storage_options = StorageOptions(uri=str(mcap_path.parent), storage_id="mcap")
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def get_message_type(type_name: str, cache: dict[str, Any]) -> Any:
    """メッセージ型を取得（キャッシュあり）."""
    if type_name not in cache:
        from rosidl_runtime_py.utilities import get_message

        cache[type_name] = get_message(type_name)
    return cache[type_name]
