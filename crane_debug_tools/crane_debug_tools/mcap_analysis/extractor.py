"""MCAP annotation extractor for RoboCup SSL matches.

このモジュールは、MCAPファイルから人間によるアノテーション(/human_annotations)と
その前後のコンテキスト(/world_model)を抽出します。
"""

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

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
        if type_name not in self._msg_types:
            self._msg_types[type_name] = get_message(type_name)
        return self._msg_types[type_name]

    def extract_from_mcap(self, mcap_path: str | Path) -> list[AnnotationContext]:
        """
        MCAPファイルからアノテーションとコンテキストを抽出.

        Args:
            mcap_path: MCAPファイルまたはrosbag2ディレクトリのパス

        Returns:
            抽出されたアノテーションコンテキストのリスト
        """
        mcap_path = Path(mcap_path)

        # ディレクトリが指定された場合は.mcapファイルを探す
        if mcap_path.is_dir():
            mcap_files = list(mcap_path.glob("*.mcap"))
            if not mcap_files:
                raise FileNotFoundError(f"No .mcap file found in {mcap_path}")
            if len(mcap_files) > 1:
                logger.warning(f"Multiple .mcap files found, using {mcap_files[0]}")
            mcap_path = mcap_files[0]

        logger.info(f"Reading MCAP file: {mcap_path}")

        # rosbag2_pyを使用してMCAPを読み込み
        try:
            from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

            storage_options = StorageOptions(
                uri=str(mcap_path.parent), storage_id="mcap"
            )
            converter_options = ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            )

            reader = SequentialReader()
            reader.open(storage_options, converter_options)

            # トピックフィルタリング
            topic_types = reader.get_all_topics_and_types()
            topics_map = {t.name: t.type for t in topic_types}

            annotations_topic = "/human_annotations"
            world_model_topic = "/world_model"

            if annotations_topic not in topics_map:
                raise ValueError(f"Topic {annotations_topic} not found in bag")
            if world_model_topic not in topics_map:
                raise ValueError(f"Topic {world_model_topic} not found in bag")

            # 1st pass: アノテーションを収集
            logger.info("Collecting annotations...")
            annotations_raw: list[tuple[int, Any]] = []

            while reader.has_next():
                topic, data, timestamp = reader.read_next()
                if topic == annotations_topic:
                    msg_type = self._get_message_type(topics_map[topic])
                    msg = deserialize_message(data, msg_type)
                    annotations_raw.append((timestamp, msg))

            logger.info(f"Found {len(annotations_raw)} annotations")

            # 2nd pass: 各アノテーションに対してWorldModelコンテキストを収集
            logger.info("Collecting WorldModel context...")
            annotations_with_context: list[AnnotationContext] = []

            # リーダーをリセット
            reader = SequentialReader()
            reader.open(storage_options, converter_options)

            for annotation_time, annotation_msg in annotations_raw:
                context = self._extract_annotation_context(
                    annotation_msg,
                    annotation_time,
                    reader,
                    topics_map,
                    world_model_topic,
                )
                annotations_with_context.append(context)

                # 各アノテーション処理後、リーダーをリセット
                reader = SequentialReader()
                reader.open(storage_options, converter_options)

            logger.info(
                f"Extracted {len(annotations_with_context)} annotations with context"
            )
            return annotations_with_context

        except ImportError as e:
            raise ImportError(
                "rosbag2_py is required. Install ROS 2 rosbag2 packages."
            ) from e

    def _extract_annotation_context(
        self,
        annotation_msg: Any,
        annotation_time: int,
        reader: Any,
        topics_map: dict[str, str],
        world_model_topic: str,
    ) -> AnnotationContext:
        """個別アノテーションのコンテキストを抽出."""
        # アノテーション基本情報を抽出
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

        # WorldModelコンテキストを収集
        context_start = event_time_ns - self.context_before_ns
        context_end = event_time_ns + self.context_after_ns

        world_model_snapshots: list[WorldModelSnapshot] = []
        last_sampled_time = 0

        msg_type = self._get_message_type(topics_map[world_model_topic])

        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic != world_model_topic:
                continue

            # コンテキスト範囲外ならスキップ
            if timestamp < context_start:
                continue
            if timestamp > context_end:
                break

            # サンプリング間隔チェック
            if timestamp - last_sampled_time < self.sampling_interval_ns:
                continue

            last_sampled_time = timestamp

            # WorldModelをデシリアライズ
            world_msg = deserialize_message(data, msg_type)

            # スナップショットを作成
            snapshot = self._create_world_model_snapshot(timestamp, world_msg)
            world_model_snapshots.append(snapshot)

        logger.debug(
            f"Collected {len(world_model_snapshots)} WorldModel snapshots for annotation at {event_time_ns}"
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
