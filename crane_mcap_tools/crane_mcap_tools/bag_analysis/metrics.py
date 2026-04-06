"""距離・速度計算ユーティリティ."""

import math
from typing import Any


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """2点間のユークリッド距離."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def speed_2d(vx: float, vy: float) -> float:
    """2次元速度ベクトルの大きさ."""
    return math.sqrt(vx**2 + vy**2)


def speed_3d(vx: float, vy: float, vz: float) -> float:
    """3次元速度ベクトルの大きさ."""
    return math.sqrt(vx**2 + vy**2 + vz**2)


def angle_diff(a1: float, a2: float) -> float:
    """2つの角度の差（-pi～pi に正規化）."""
    diff = a2 - a1
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def ball_to_robot_dist(
    world_msg: Any, robot_id: int, is_ours: bool = True
) -> float | None:
    """WorldModelメッセージから特定ロボットとボールの距離を計算.

    Returns:
        距離（m）。ロボットが見つからない場合は None。
    """
    ball = world_msg.ball_info
    bx, by = ball.position.x, ball.position.y

    robots = world_msg.robot_info_ours if is_ours else world_msg.robot_info_theirs
    for r in robots:
        if r.id == robot_id:
            return distance_2d(r.pose.x, r.pose.y, bx, by)
    return None
