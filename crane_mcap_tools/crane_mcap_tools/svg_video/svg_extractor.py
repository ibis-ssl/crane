"""MCAP SVG extractor for RoboCup SSL matches.

このモジュールは、MCAPファイルから/aggregated_svgsと/visualizer_svgsトピックを読み込み、
タイムスタンプ順にマージしてレイヤー状態を累積管理します。
"""

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

from rclpy.serialization import deserialize_message

from ..mcap_utils import get_message_type, open_sequential_reader, resolve_mcap_path

logger = logging.getLogger(__name__)


@dataclass
class SvgFrame:
    """単一フレームのSVGレイヤー状態."""

    timestamp_ns: int
    epoch: int
    seq: int
    layers: dict[str, list[str]] = field(
        default_factory=dict
    )  # layer_name -> svg_primitives


class SvgExtractor:
    """MCAPファイルからSVGメッセージを抽出."""

    def __init__(
        self,
        snapshot_topic: str = "/aggregated_svgs",
        update_topic: str = "/visualizer_svgs",
    ):
        """
        初期化.

        Args:
            snapshot_topic: スナップショットトピック名
            update_topic: 増分更新トピック名
        """
        self.snapshot_topic = snapshot_topic
        self.update_topic = update_topic

        # メッセージ型キャッシュ
        self._msg_types: dict[str, Any] = {}

    def _get_message_type(self, type_name: str) -> Any:
        """メッセージ型を取得（キャッシュあり）."""
        return get_message_type(type_name, self._msg_types)

    def extract_from_mcap(
        self,
        mcap_path: str | Path,
        start_time_sec: float | None = None,
        end_time_sec: float | None = None,
    ) -> Iterator[SvgFrame]:
        """
        MCAPファイルからSVGフレームを抽出.

        Args:
            mcap_path: MCAPファイルまたはrosbag2ディレクトリのパス
            start_time_sec: 開始時刻（秒、Noneの場合は先頭から）
            end_time_sec: 終了時刻（秒、Noneの場合は最後まで）

        Yields:
            抽出されたSVGフレーム（タイムスタンプ順）
        """
        mcap_path = resolve_mcap_path(Path(mcap_path))

        logger.info(f"Reading MCAP file: {mcap_path}")

        # 時刻範囲をナノ秒に変換
        start_time_ns = (
            int(start_time_sec * 1e9) if start_time_sec is not None else None
        )
        end_time_ns = int(end_time_sec * 1e9) if end_time_sec is not None else None

        # rosbag2_pyを使用してMCAPを読み込み
        try:
            reader = open_sequential_reader(mcap_path)

            # トピックフィルタリング
            topic_types = reader.get_all_topics_and_types()
            topics_map = {t.name: t.type for t in topic_types}

            # トピックの存在確認（どちらかあればOK）
            has_snapshot = self.snapshot_topic in topics_map
            has_update = self.update_topic in topics_map

            if not has_snapshot and not has_update:
                raise ValueError(
                    f"Neither {self.snapshot_topic} nor {self.update_topic} found in bag"
                )

            if has_snapshot:
                logger.info(f"Found snapshot topic: {self.snapshot_topic}")
            if has_update:
                logger.info(f"Found update topic: {self.update_topic}")

            # メッセージを収集してタイムスタンプ順にソート
            logger.info("Collecting all SVG messages...")
            messages: list[tuple[int, str, Any]] = []  # (timestamp, topic, msg)

            while reader.has_next():
                topic, data, timestamp = reader.read_next()

                # 時刻フィルタリング
                if start_time_ns is not None and timestamp < start_time_ns:
                    continue
                if end_time_ns is not None and timestamp > end_time_ns:
                    break

                if topic in [self.snapshot_topic, self.update_topic]:
                    msg_type = self._get_message_type(topics_map[topic])
                    msg = deserialize_message(data, msg_type)
                    messages.append((timestamp, topic, msg))

            logger.info(f"Collected {len(messages)} messages")

            # タイムスタンプ順にソート
            messages.sort(key=lambda x: x[0])

            # 累積レイヤー状態
            current_layers: dict[str, list[str]] = {}
            last_epoch = 0

            # メッセージを処理してフレームを生成
            for timestamp, topic, msg in messages:
                epoch = msg.epoch
                seq = msg.seq

                # epoch更新時は状態をリセット
                if epoch != last_epoch:
                    logger.info(
                        f"Epoch changed: {last_epoch} -> {epoch}, resetting state"
                    )
                    current_layers = {}

                last_epoch = epoch

                if topic == self.snapshot_topic:
                    # スナップショット: 全レイヤーを置換
                    current_layers = {}
                    for layer_snapshot in msg.layers:
                        layer_name = layer_snapshot.layer
                        primitives = list(layer_snapshot.svg_primitives)
                        current_layers[layer_name] = primitives

                elif topic == self.update_topic:
                    # 増分更新: 各レイヤーに操作を適用
                    for update in msg.updates:
                        layer_name = update.layer
                        operation = update.operation.lower()
                        primitives = list(update.svg_primitives)

                        if operation == "replace":
                            current_layers[layer_name] = primitives
                        elif operation == "append":
                            if layer_name in current_layers:
                                current_layers[layer_name].extend(primitives)
                            # ベースがない場合のappendは無視（svg_viewer.jsの仕様に準拠）
                        elif operation == "clear":
                            current_layers[layer_name] = []
                        else:
                            logger.warning(f"Unknown operation: {operation}")

                # フレームをyield（現在の累積状態のコピー）
                yield SvgFrame(
                    timestamp_ns=timestamp,
                    epoch=epoch,
                    seq=seq,
                    layers={k: list(v) for k, v in current_layers.items()},
                )

        except ImportError as e:
            raise ImportError(
                "rosbag2_py is required. Install ROS 2 rosbag2 packages."
            ) from e
