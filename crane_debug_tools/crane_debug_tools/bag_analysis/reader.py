"""MCAPファイル読み込みエンジン."""

from pathlib import Path
from typing import Any

from rclpy.serialization import deserialize_message

from ..mcap_utils import get_message_type, open_sequential_reader, resolve_mcap_path
from .models import BagData, BagInfo, TimestampedMsg

# 標準7トピック
DEFAULT_TOPICS = [
    "/play_situation",
    "/world_model",
    "/control_targets",
    "/robot_commands",
    "/game_analysis",
    "/robot_select_results",
    "/rosout",
]


class BagReader:
    """MCAPファイル/rosbagディレクトリを読み込むエンジン."""

    def __init__(self, bag_path: str | Path):
        self.bag_path = Path(bag_path)
        self._msg_types: dict[str, Any] = {}

    def _get_message_type(self, type_name: str) -> Any:
        """メッセージ型を取得（キャッシュあり）."""
        return get_message_type(type_name, self._msg_types)

    def _resolve_mcap_path(self) -> Path:
        """MCAPファイルのパスを解決する."""
        return resolve_mcap_path(self.bag_path)

    def _open_reader(self, mcap_file: Path):
        """SequentialReaderを開く."""
        return open_sequential_reader(mcap_file)

    def info(self) -> BagInfo:
        """Bagのメタデータのみを取得する."""
        mcap_file = self._resolve_mcap_path()
        reader = self._open_reader(mcap_file)

        topic_types_list = reader.get_all_topics_and_types()
        topic_types = {t.name: t.type for t in topic_types_list}

        topic_counts: dict[str, int] = {}
        start_ns = None
        end_ns = None

        while reader.has_next():
            topic, _data, timestamp = reader.read_next()
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
            if start_ns is None or timestamp < start_ns:
                start_ns = timestamp
            if end_ns is None or timestamp > end_ns:
                end_ns = timestamp

        start_ns = start_ns or 0
        end_ns = end_ns or 0

        return BagInfo(
            path=str(self.bag_path),
            duration_sec=(end_ns - start_ns) / 1e9,
            start_time_ns=start_ns,
            end_time_ns=end_ns,
            topic_counts=topic_counts,
            topic_types=topic_types,
        )

    def read(
        self,
        topics: list[str] | None = None,
        time_range: tuple[float, float] | None = None,
    ) -> BagData:
        """Bagを読み込んでBagDataを返す.

        Args:
            topics: 読み込むトピックのリスト。Noneの場合は標準7トピック。
            time_range: (start_sec, end_sec) bag開始からの相対秒数。

        Returns:
            BagData
        """
        target_topics = set(topics) if topics is not None else set(DEFAULT_TOPICS)
        mcap_file = self._resolve_mcap_path()
        reader = self._open_reader(mcap_file)

        topic_types_list = reader.get_all_topics_and_types()
        topic_types = {t.name: t.type for t in topic_types_list}

        # 最初にbag全体のstart/endを取得するため、1パスで処理
        raw_messages: list[tuple[str, bytes, int]] = []
        start_ns_all: int | None = None
        end_ns_all: int | None = None
        topic_counts_all: dict[str, int] = {}

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            topic_counts_all[topic] = topic_counts_all.get(topic, 0) + 1
            if start_ns_all is None or timestamp < start_ns_all:
                start_ns_all = timestamp
            if end_ns_all is None or timestamp > end_ns_all:
                end_ns_all = timestamp
            if topic in target_topics:
                raw_messages.append((topic, data, timestamp))

        start_ns = start_ns_all or 0
        end_ns = end_ns_all or 0

        # 時間範囲フィルタ
        if time_range is not None:
            range_start_ns = start_ns + int(time_range[0] * 1e9)
            range_end_ns = start_ns + int(time_range[1] * 1e9)
        else:
            range_start_ns = None
            range_end_ns = None

        # デシリアライズ
        messages: dict[str, list[TimestampedMsg]] = {t: [] for t in target_topics}

        for topic, data, timestamp in raw_messages:
            if range_start_ns is not None and timestamp < range_start_ns:
                continue
            if range_end_ns is not None and timestamp > range_end_ns:
                continue
            msg_type = self._get_message_type(topic_types[topic])
            msg = deserialize_message(data, msg_type)
            messages[topic].append(
                TimestampedMsg(
                    timestamp_ns=timestamp,
                    msg=msg,
                    _bag_start_ns=start_ns,
                )
            )

        bag_info = BagInfo(
            path=str(self.bag_path),
            duration_sec=(end_ns - start_ns) / 1e9,
            start_time_ns=start_ns,
            end_time_ns=end_ns,
            topic_counts=topic_counts_all,
            topic_types=topic_types,
        )

        return BagData(info=bag_info, messages=messages)
