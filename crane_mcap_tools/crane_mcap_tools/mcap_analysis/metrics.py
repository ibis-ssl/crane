"""距離・速度計算ユーティリティ."""

import math


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """2点間のユークリッド距離."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def speed_2d(vx: float, vy: float) -> float:
    """2次元速度ベクトルの大きさ."""
    return math.sqrt(vx**2 + vy**2)


def speed_3d(vx: float, vy: float, vz: float) -> float:
    """3次元速度ベクトルの大きさ."""
    return math.sqrt(vx**2 + vy**2 + vz**2)
