"""BagAnalysisのデータモデル定義."""

from dataclasses import dataclass, field
from typing import Any


@dataclass
class BagInfo:
    """Bagファイルのメタデータ."""

    path: str
    duration_sec: float
    start_time_ns: int
    end_time_ns: int
    topic_counts: dict[str, int]
    topic_types: dict[str, str]


@dataclass
class TimestampedMsg:
    """タイムスタンプ付きROSメッセージ."""

    timestamp_ns: int
    msg: Any
    _bag_start_ns: int = field(default=0, repr=False)

    @property
    def t(self) -> float:
        """Bag開始からの相対秒数."""
        return (self.timestamp_ns - self._bag_start_ns) / 1e9


@dataclass
class BagData:
    """読み込んだBagデータのコンテナ."""

    info: BagInfo
    messages: dict[str, list[TimestampedMsg]]

    def get(self, topic: str) -> list[TimestampedMsg]:
        """指定トピックのメッセージリストを取得."""
        return self.messages.get(topic, [])

    def time_slice(self, start_sec: float, end_sec: float) -> "BagData":
        """時間範囲でフィルタリングした新しいBagDataを返す."""
        start_ns = self.info.start_time_ns + int(start_sec * 1e9)
        end_ns = self.info.start_time_ns + int(end_sec * 1e9)
        filtered: dict[str, list[TimestampedMsg]] = {}
        for topic, msgs in self.messages.items():
            filtered[topic] = [m for m in msgs if start_ns <= m.timestamp_ns <= end_ns]
        return BagData(info=self.info, messages=filtered)

    def sample(self, topic: str, interval_sec: float) -> list[TimestampedMsg]:
        """指定間隔でサンプリングしたメッセージリストを返す."""
        msgs = self.get(topic)
        if not msgs:
            return []
        result: list[TimestampedMsg] = []
        last_ns = 0
        interval_ns = int(interval_sec * 1e9)
        for m in msgs:
            if m.timestamp_ns - last_ns >= interval_ns:
                result.append(m)
                last_ns = m.timestamp_ns
        return result
