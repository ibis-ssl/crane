"""MCAP data access tools for Gemini Function Calling.

このモジュールは、Geminiが自律的にMCAPデータを探索できるツール関数群を提供します。
"""

import logging
from typing import Any

from .extractor import AnnotationContext, WorldModelSnapshot
from ..bag_analysis.metrics import distance_2d, speed_2d, speed_3d

logger = logging.getLogger(__name__)


class MCAPToolsHandler:
    """Gemini Function Calling用のMCAPデータアクセスツール."""

    def __init__(self, annotation: AnnotationContext):
        """
        初期化.

        Args:
            annotation: 現在分析中のアノテーションコンテキスト
        """
        self.annotation = annotation
        self.world_model_snapshots = annotation.world_model_context

    def handle(self, function_name: str, args: dict[str, Any]) -> dict[str, Any]:
        """
        関数呼び出しをハンドル.

        Args:
            function_name: 関数名
            args: 関数引数

        Returns:
            実行結果
        """
        handlers = {
            "get_world_model_at_time": self._get_world_model_at_time,
            "get_robot_trajectory": self._get_robot_trajectory,
            "get_ball_trajectory": self._get_ball_trajectory,
            "calculate_distance": self._calculate_distance,
            "get_robot_speed_history": self._get_robot_speed_history,
            "find_closest_robot_to_ball": self._find_closest_robot_to_ball,
            "get_ball_speed_history": self._get_ball_speed_history,
            "check_robot_collision": self._check_robot_collision,
        }

        handler = handlers.get(function_name)
        if handler:
            try:
                logger.debug(f"Calling function: {function_name} with args: {args}")
                result = handler(args)
                logger.debug(f"Function result: {result}")
                return result
            except Exception as e:
                logger.error(f"Error in {function_name}: {e}")
                return {"error": str(e)}
        else:
            return {"error": f"Unknown function: {function_name}"}

    def _get_world_model_at_time(self, args: dict[str, Any]) -> dict[str, Any]:
        """
        特定時刻のWorldModelスナップショットを取得.

        Args:
            time_offset_sec: イベント時刻からのオフセット（秒）
        """
        time_offset_sec = args.get("time_offset_sec", 0.0)
        target_time_ns = self.annotation.event_timestamp_ns + int(time_offset_sec * 1e9)

        # 最も近いスナップショットを検索
        closest_snapshot = min(
            self.world_model_snapshots,
            key=lambda s: abs(s.timestamp_ns - target_time_ns),
        )

        return {
            "timestamp_sec": closest_snapshot.timestamp_ns / 1e9,
            "ball_position": closest_snapshot.ball_position,
            "ball_velocity": closest_snapshot.ball_velocity,
            "our_robots": closest_snapshot.our_robots,
            "their_robots": closest_snapshot.their_robots,
        }

    def _get_robot_trajectory(self, args: dict[str, Any]) -> dict[str, Any]:
        """
        ロボットの軌跡を取得.

        Args:
            robot_id: ロボットID
            is_ours: 自チームかどうか
        """
        robot_id = args.get("robot_id")
        is_ours = args.get("is_ours", True)

        trajectory = []
        for snapshot in self.world_model_snapshots:
            robots = snapshot.our_robots if is_ours else snapshot.their_robots
            robot_data = next((r for r in robots if r["id"] == robot_id), None)

            if robot_data:
                trajectory.append(
                    {
                        "timestamp_sec": snapshot.timestamp_ns / 1e9,
                        "position": robot_data["position"],
                        "velocity": robot_data["velocity"],
                        "theta": robot_data["theta"],
                    }
                )

        return {
            "robot_id": robot_id,
            "is_ours": is_ours,
            "trajectory": trajectory,
            "total_distance": self._calculate_trajectory_distance(trajectory),
        }

    def _get_ball_trajectory(self, args: dict[str, Any]) -> dict[str, Any]:
        """ボールの軌跡を取得."""
        trajectory = []
        for snapshot in self.world_model_snapshots:
            trajectory.append(
                {
                    "timestamp_sec": snapshot.timestamp_ns / 1e9,
                    "position": snapshot.ball_position,
                    "velocity": snapshot.ball_velocity,
                }
            )

        return {
            "trajectory": trajectory,
            "total_distance": self._calculate_trajectory_distance(trajectory),
            "max_speed": max(
                (speed_3d(*v) for v in [t["velocity"] for t in trajectory]),
                default=0.0,
            ),
        }

    def _calculate_distance(self, args: dict[str, Any]) -> dict[str, Any]:
        """
        2点間の距離を計算.

        Args:
            point1: [x, y] or [x, y, z]
            point2: [x, y] or [x, y, z]
        """
        point1 = args.get("point1", [])
        point2 = args.get("point2", [])

        if len(point1) < 2 or len(point2) < 2:
            return {"error": "Invalid points"}

        return {"distance": distance_2d(point1[0], point1[1], point2[0], point2[1])}

    def _get_robot_speed_history(self, args: dict[str, Any]) -> dict[str, Any]:
        """
        ロボットの速度履歴を取得.

        Args:
            robot_id: ロボットID
            is_ours: 自チームかどうか
        """
        robot_id = args.get("robot_id")
        is_ours = args.get("is_ours", True)

        speed_history = []
        for snapshot in self.world_model_snapshots:
            robots = snapshot.our_robots if is_ours else snapshot.their_robots
            robot_data = next((r for r in robots if r["id"] == robot_id), None)

            if robot_data:
                vx, vy = robot_data["velocity"]
                speed_history.append(
                    {
                        "timestamp_sec": snapshot.timestamp_ns / 1e9,
                        "speed": speed_2d(vx, vy),
                    }
                )

        if speed_history:
            return {
                "robot_id": robot_id,
                "is_ours": is_ours,
                "speed_history": speed_history,
                "max_speed": max(s["speed"] for s in speed_history),
                "min_speed": min(s["speed"] for s in speed_history),
                "avg_speed": sum(s["speed"] for s in speed_history)
                / len(speed_history),
            }
        else:
            return {"error": "Robot not found in snapshots"}

    def _find_closest_robot_to_ball(self, args: dict[str, Any]) -> dict[str, Any]:
        """
        ボールに最も近いロボットを検索.

        Args:
            time_offset_sec: イベント時刻からのオフセット（秒）
            team: "ours", "theirs", "both"
        """
        time_offset_sec = args.get("time_offset_sec", 0.0)
        team = args.get("team", "both")

        snapshot = self._get_snapshot_at_offset(time_offset_sec)
        if not snapshot:
            return {"error": "No snapshot found"}

        ball_pos = snapshot.ball_position

        # 対象ロボット群を決定
        if team == "ours":
            robots = [(r, True) for r in snapshot.our_robots]
        elif team == "theirs":
            robots = [(r, False) for r in snapshot.their_robots]
        else:  # both
            robots = [(r, True) for r in snapshot.our_robots] + [
                (r, False) for r in snapshot.their_robots
            ]

        if not robots:
            return {"error": "No robots found"}

        # 最近接ロボットを検索
        closest_robot = None
        min_distance = float("inf")

        for robot, is_ours in robots:
            rx, ry = robot["position"]
            distance = distance_2d(ball_pos[0], ball_pos[1], rx, ry)

            if distance < min_distance:
                min_distance = distance
                closest_robot = (robot, is_ours)

        if closest_robot:
            robot, is_ours = closest_robot
            return {
                "robot_id": robot["id"],
                "is_ours": is_ours,
                "distance": min_distance,
                "position": robot["position"],
                "velocity": robot["velocity"],
            }
        else:
            return {"error": "No closest robot found"}

    def _get_ball_speed_history(self, args: dict[str, Any]) -> dict[str, Any]:
        """ボールの速度履歴を取得."""
        speed_history = []
        for snapshot in self.world_model_snapshots:
            vx, vy, vz = snapshot.ball_velocity
            speed_history.append(
                {
                    "timestamp_sec": snapshot.timestamp_ns / 1e9,
                    "speed": speed_3d(vx, vy, vz),
                }
            )

        if speed_history:
            return {
                "speed_history": speed_history,
                "max_speed": max(s["speed"] for s in speed_history),
                "min_speed": min(s["speed"] for s in speed_history),
                "avg_speed": sum(s["speed"] for s in speed_history)
                / len(speed_history),
            }
        else:
            return {"error": "No ball data"}

    def _check_robot_collision(self, args: dict[str, Any]) -> dict[str, Any]:
        """
        ロボット間の接近・衝突をチェック.

        Args:
            threshold_distance: 衝突判定距離（メートル）
        """
        threshold = args.get("threshold_distance", 0.3)  # 30cm

        collisions = []
        for snapshot in self.world_model_snapshots:
            # 自チーム内
            for i, r1 in enumerate(snapshot.our_robots):
                for r2 in snapshot.our_robots[i + 1 :]:
                    distance = distance_2d(
                        r1["position"][0],
                        r1["position"][1],
                        r2["position"][0],
                        r2["position"][1],
                    )
                    if distance < threshold:
                        collisions.append(
                            {
                                "timestamp_sec": snapshot.timestamp_ns / 1e9,
                                "robot1_id": r1["id"],
                                "robot2_id": r2["id"],
                                "both_ours": True,
                                "distance": distance,
                            }
                        )

            # 自チームと相手チーム
            for r1 in snapshot.our_robots:
                for r2 in snapshot.their_robots:
                    distance = distance_2d(
                        r1["position"][0],
                        r1["position"][1],
                        r2["position"][0],
                        r2["position"][1],
                    )
                    if distance < threshold:
                        collisions.append(
                            {
                                "timestamp_sec": snapshot.timestamp_ns / 1e9,
                                "our_robot_id": r1["id"],
                                "their_robot_id": r2["id"],
                                "both_ours": False,
                                "distance": distance,
                            }
                        )

        return {"collisions": collisions, "collision_count": len(collisions)}

    # ヘルパー関数

    def _get_snapshot_at_offset(
        self, time_offset_sec: float
    ) -> WorldModelSnapshot | None:
        """オフセット時刻のスナップショットを取得."""
        target_time_ns = self.annotation.event_timestamp_ns + int(time_offset_sec * 1e9)
        if not self.world_model_snapshots:
            return None

        return min(
            self.world_model_snapshots,
            key=lambda s: abs(s.timestamp_ns - target_time_ns),
        )

    def _calculate_trajectory_distance(self, trajectory: list[dict]) -> float:
        """軌跡の総移動距離を計算."""
        if len(trajectory) < 2:
            return 0.0

        total = 0.0
        for i in range(len(trajectory) - 1):
            p1 = trajectory[i]["position"]
            p2 = trajectory[i + 1]["position"]
            total += distance_2d(p1[0], p1[1], p2[0], p2[1])
        return total


# Gemini Function Calling用のツール定義スキーマ
MCAP_TOOLS_SCHEMA = [
    {
        "name": "get_world_model_at_time",
        "description": "特定時刻のWorldModel（ボール位置、全ロボット状態）を取得します。イベント時刻からの相対時刻で指定します。",
        "parameters": {
            "type": "object",
            "properties": {
                "time_offset_sec": {
                    "type": "number",
                    "description": "イベント時刻からのオフセット（秒）。負の値で過去、正の値で未来を指定。例: -1.0 は1秒前",
                }
            },
            "required": ["time_offset_sec"],
        },
    },
    {
        "name": "get_robot_trajectory",
        "description": "指定ロボットの軌跡（位置・速度の時系列データ）を取得します。",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "integer", "description": "ロボットID（0-15）"},
                "is_ours": {
                    "type": "boolean",
                    "description": "自チームの場合true、相手チームの場合false",
                },
            },
            "required": ["robot_id", "is_ours"],
        },
    },
    {
        "name": "get_ball_trajectory",
        "description": "ボールの軌跡（位置・速度の時系列データ）を取得します。",
        "parameters": {"type": "object", "properties": {}},
    },
    {
        "name": "calculate_distance",
        "description": "2点間の距離を計算します。",
        "parameters": {
            "type": "object",
            "properties": {
                "point1": {
                    "type": "array",
                    "items": {"type": "number"},
                    "description": "第1点の座標 [x, y]",
                },
                "point2": {
                    "type": "array",
                    "items": {"type": "number"},
                    "description": "第2点の座標 [x, y]",
                },
            },
            "required": ["point1", "point2"],
        },
    },
    {
        "name": "get_robot_speed_history",
        "description": "指定ロボットの速度履歴を取得します。",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_id": {"type": "integer", "description": "ロボットID（0-15）"},
                "is_ours": {
                    "type": "boolean",
                    "description": "自チームの場合true、相手チームの場合false",
                },
            },
            "required": ["robot_id", "is_ours"],
        },
    },
    {
        "name": "find_closest_robot_to_ball",
        "description": "指定時刻でボールに最も近いロボットを検索します。",
        "parameters": {
            "type": "object",
            "properties": {
                "time_offset_sec": {
                    "type": "number",
                    "description": "イベント時刻からのオフセット（秒）",
                },
                "team": {
                    "type": "string",
                    "enum": ["ours", "theirs", "both"],
                    "description": "対象チーム",
                },
            },
            "required": ["time_offset_sec", "team"],
        },
    },
    {
        "name": "get_ball_speed_history",
        "description": "ボールの速度履歴を取得します。",
        "parameters": {"type": "object", "properties": {}},
    },
    {
        "name": "check_robot_collision",
        "description": "ロボット間の接近・衝突をチェックします。",
        "parameters": {
            "type": "object",
            "properties": {
                "threshold_distance": {
                    "type": "number",
                    "description": "衝突判定距離（メートル）。デフォルト0.3m",
                }
            },
        },
    },
]
