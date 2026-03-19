"""MCAP annotation extractor for RoboCup SSL matches.

このモジュールは、MCAPファイルから人間によるアノテーション(/human_annotations)と
その前後のコンテキスト(/world_model)を抽出します。
"""

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rclpy.serialization import deserialize_message

from ..mcap_utils import get_message_type, open_sequential_reader, resolve_mcap_path

logger = logging.getLogger(__name__)


@dataclass
class WorldModelSnapshot:
    """WorldModelのスナップショット."""

    timestamp_ns: int
    ball_position: tuple[float, float, float]
    ball_velocity: tuple[float, float, float]
    our_robots: list[dict[str, Any]]
    their_robots: list[dict[str, Any]]


@dataclass
class AnnotationContext:
    """アノテーションとその前後のコンテキスト."""

    # アノテーション基本情報
    category: int
    priority: int
    label: str
    description: str
    event_timestamp_ns: int

    # 位置情報
    has_position: bool
    position: tuple[float, float, float] | None = None

    # ロボットコンテキスト
    has_robot_context: bool = False
    related_robot_ids: list[int] = field(default_factory=list)
    robot_is_ours: list[bool] = field(default_factory=list)

    # WorldModelコンテキスト（前後3秒+2秒のスナップショット）
    world_model_context: list[WorldModelSnapshot] = field(default_factory=list)

    # カテゴリ名マッピング
    CATEGORY_NAMES = {
        0: "ISSUE",
        1: "OBSERVATION",
        2: "QUESTION",
        3: "POSITIVE",
        4: "STRATEGY",
        5: "TIMING",
        10: "CUSTOM",
    }

    # 重要度名マッピング
    PRIORITY_NAMES = {
        0: "LOW",
        1: "MEDIUM",
        2: "HIGH",
        3: "CRITICAL",
    }

    def get_category_name(self) -> str:
        """カテゴリ名を取得."""
        return self.CATEGORY_NAMES.get(self.category, "UNKNOWN")

    def get_priority_name(self) -> str:
        """重要度名を取得."""
        return self.PRIORITY_NAMES.get(self.priority, "UNKNOWN")


class MCAPAnnotationExtractor:
    """MCAPファイルからアノテーションとコンテキストを抽出."""

    def __init__(
        self,
        context_before_sec: float = 3.0,
        context_after_sec: float = 2.0,
        world_model_sampling_ms: int = 100,
    ):
        """
        初期化.

        Args:
            context_before_sec: アノテーション前のコンテキスト時間（秒）
            context_after_sec: アノテーション後のコンテキスト時間（秒）
            world_model_sampling_ms: WorldModelサンプリング間隔（ミリ秒）
        """
        self.context_before_ns = int(context_before_sec * 1e9)
        self.context_after_ns = int(context_after_sec * 1e9)
        self.sampling_interval_ns = int(world_model_sampling_ms * 1e6)

        # メッセージ型キャッシュ
        self._msg_types: dict[str, Any] = {}

    def _get_message_type(self, type_name: str) -> Any:
        """メッセージ型を取得（キャッシュあり）."""
        return get_message_type(type_name, self._msg_types)

    def extract_from_mcap(self, mcap_path: str | Path) -> list[AnnotationContext]:
        """
        MCAPファイルからアノテーションとコンテキストを抽出.

        Args:
            mcap_path: MCAPファイルまたはrosbag2ディレクトリのパス

        Returns:
            抽出されたアノテーションコンテキストのリスト
        """
        mcap_path = resolve_mcap_path(Path(mcap_path))

        logger.info(f"Reading MCAP file: {mcap_path}")

        reader = open_sequential_reader(mcap_path)

        # トピックフィルタリング
        topic_types = reader.get_all_topics_and_types()
        topics_map = {t.name: t.type for t in topic_types}

        annotations_topic = "/human_annotations"
        world_model_topic = "/world_model"

        if annotations_topic not in topics_map:
            raise ValueError(f"Topic {annotations_topic} not found in bag")
        if world_model_topic not in topics_map:
            raise ValueError(f"Topic {world_model_topic} not found in bag")

        # 1パスでアノテーションとWorldModelを収集
        logger.info("Collecting annotations and WorldModel messages...")
        annotations_raw: list[tuple[int, Any]] = []
        world_model_messages: list[tuple[int, Any]] = []

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == annotations_topic:
                msg_type = self._get_message_type(topics_map[topic])
                msg = deserialize_message(data, msg_type)
                annotations_raw.append((timestamp, msg))
            elif topic == world_model_topic:
                msg_type = self._get_message_type(topics_map[topic])
                msg = deserialize_message(data, msg_type)
                world_model_messages.append((timestamp, msg))

        logger.info(
            f"Found {len(annotations_raw)} annotations, "
            f"{len(world_model_messages)} WorldModel messages"
        )

        # 各アノテーションに対してメモリ上でWorldModelコンテキストをフィルタリング
        logger.info("Building annotation contexts...")
        annotations_with_context = [
            self._build_annotation_context(
                annotation_msg, annotation_time, world_model_messages
            )
            for annotation_time, annotation_msg in annotations_raw
        ]

        logger.info(
            f"Extracted {len(annotations_with_context)} annotations with context"
        )
        return annotations_with_context

    def _build_annotation_context(
        self,
        annotation_msg: Any,
        annotation_time: int,
        world_model_messages: list[tuple[int, Any]],
    ) -> AnnotationContext:
        """メモリ上のWorldModelリストからアノテーションコンテキストを構築."""
        event_time_ns = annotation_msg.event_timestamp_ns

        # 位置情報
        position = None
        if annotation_msg.has_position:
            pos = annotation_msg.position
            position = (pos.x, pos.y, pos.z)

        # ロボットコンテキスト
        related_robot_ids = (
            list(annotation_msg.related_robot_ids)
            if annotation_msg.has_robot_context
            else []
        )
        robot_is_ours = (
            list(annotation_msg.robot_is_ours)
            if annotation_msg.has_robot_context
            else []
        )

        # WorldModelコンテキストをフィルタリング（時間範囲 + サンプリング）
        context_start = event_time_ns - self.context_before_ns
        context_end = event_time_ns + self.context_after_ns

        world_model_snapshots: list[WorldModelSnapshot] = []
        last_sampled_time = 0

        for timestamp, world_msg in world_model_messages:
            if timestamp < context_start:
                continue
            if timestamp > context_end:
                continue
            if timestamp - last_sampled_time < self.sampling_interval_ns:
                continue

            last_sampled_time = timestamp
            world_model_snapshots.append(
                self._create_world_model_snapshot(timestamp, world_msg)
            )

        logger.debug(
            f"Collected {len(world_model_snapshots)} WorldModel snapshots "
            f"for annotation at {event_time_ns}"
        )

        return AnnotationContext(
            category=annotation_msg.category,
            priority=annotation_msg.priority,
            label=annotation_msg.label,
            description=annotation_msg.description,
            event_timestamp_ns=event_time_ns,
            has_position=annotation_msg.has_position,
            position=position,
            has_robot_context=annotation_msg.has_robot_context,
            related_robot_ids=related_robot_ids,
            robot_is_ours=robot_is_ours,
            world_model_context=world_model_snapshots,
        )

    def _create_world_model_snapshot(
        self, timestamp: int, world_msg: Any
    ) -> WorldModelSnapshot:
        """WorldModelメッセージからスナップショットを作成."""
        # ボール情報
        ball_info = world_msg.ball_info
        ball_pos = ball_info.position
        ball_vel = ball_info.velocity

        # ロボット情報
        our_robots = [
            {
                "id": robot.id,
                "position": (robot.pose.x, robot.pose.y),
                "theta": robot.pose.theta,
                "velocity": (robot.velocity.x, robot.velocity.y),
                "has_error": robot.has_error,
            }
            for robot in world_msg.robot_info_ours
        ]

        their_robots = [
            {
                "id": robot.id,
                "position": (robot.pose.x, robot.pose.y),
                "theta": robot.pose.theta,
                "velocity": (robot.velocity.x, robot.velocity.y),
            }
            for robot in world_msg.robot_info_theirs
        ]

        return WorldModelSnapshot(
            timestamp_ns=timestamp,
            ball_position=(ball_pos.x, ball_pos.y, ball_pos.z),
            ball_velocity=(ball_vel.x, ball_vel.y, ball_vel.z),
            our_robots=our_robots,
            their_robots=their_robots,
        )
