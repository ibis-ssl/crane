"""JSON軌道データ読み込みモジュール (C++ BallCalibrationDataExtractor の移植)."""

from __future__ import annotations

import json
import logging
import math
import re
from pathlib import Path

from .models import TrajectoryData

logger = logging.getLogger(__name__)

# JSONファイルパターン
_JSON_PATTERN = re.compile(r"kick_event_visualization_(\d+)_data\.json")

# ボール状態定数
_BALL_STATE_ROLLING = 1


def load_trajectory_from_json(file_path: str | Path) -> TrajectoryData | None:
    """JSONファイルから1件の軌道データを読み込む.

    C++ loadTrajectoryDataFromJSON の移植。ball_state==1(ROLLING) のみ抽出。
    """
    file_path = Path(file_path)
    try:
        with file_path.open() as f:
            data = json.load(f)
    except Exception:
        logger.warning("JSONファイルを開けませんでした: %s", file_path)
        return None

    trajectory = TrajectoryData()

    # ファイル名からevent_idを抽出
    match = _JSON_PATTERN.match(file_path.name)
    if match:
        trajectory.event_id = int(match.group(1))

    # キック情報
    kick_info = data.get("kick_info", {})
    trajectory.kick_power = kick_info.get("kick_power", 0.0)
    trajectory.is_chip_kick = kick_info.get("is_chip_kick", False)

    # 軌道データ
    payload = data.get("data", {})
    timestamp_array = payload.get("timestamp_ns", [])
    position = payload.get("position", {})
    pos_x_array = position.get("x", [])
    pos_y_array = position.get("y", [])
    ball_state_array = payload.get("ball_state", [])

    data_size = min(
        len(timestamp_array), len(pos_x_array), len(pos_y_array), len(ball_state_array)
    )

    # ROLLING状態のインデックスのみ抽出
    valid_indices = [
        i for i in range(data_size) if ball_state_array[i] == _BALL_STATE_ROLLING
    ]

    if not valid_indices:
        logger.debug("有効なROLLINGデータなし: %s", file_path.name)
        return None

    # ナノ秒→秒変換
    timestamps_sec = [timestamp_array[i] * 1e-9 for i in valid_indices]
    start_time = timestamps_sec[0]

    # 相対時間
    trajectory.time_points = [t - start_time for t in timestamps_sec]
    trajectory.positions_x = [pos_x_array[i] for i in valid_indices]
    trajectory.positions_y = [pos_y_array[i] for i in valid_indices]

    # 差分速度計算
    velocities = []
    n = len(trajectory.time_points)
    for i in range(n):
        if i == 0:
            if n > 1:
                dx = trajectory.positions_x[1] - trajectory.positions_x[0]
                dy = trajectory.positions_y[1] - trajectory.positions_y[0]
                dt = trajectory.time_points[1] - trajectory.time_points[0]
                speed = math.sqrt(dx * dx + dy * dy) / dt if dt > 0 else 0.0
            else:
                speed = 0.0
        else:
            dx = trajectory.positions_x[i] - trajectory.positions_x[i - 1]
            dy = trajectory.positions_y[i] - trajectory.positions_y[i - 1]
            dt = trajectory.time_points[i] - trajectory.time_points[i - 1]
            speed = math.sqrt(dx * dx + dy * dy) / dt if dt > 0 else 0.0
        velocities.append(speed)
    trajectory.velocities = velocities

    return trajectory


def load_all_trajectories(directory_path: str | Path) -> list[TrajectoryData]:
    """ディレクトリ内の全JSONファイルから軌道データを読み込む.

    C++ loadAllTrajectoryData の移植。
    """
    directory_path = Path(directory_path)
    if not directory_path.exists():
        logger.error("ディレクトリが存在しません: %s", directory_path)
        return []

    trajectories = []
    for entry in sorted(directory_path.iterdir()):
        if entry.is_file() and _JSON_PATTERN.match(entry.name):
            traj = load_trajectory_from_json(entry)
            if traj is not None and traj.time_points:
                trajectories.append(traj)

    logger.info(
        "JSONファイル読み込み完了: %d個の軌道データを抽出 (ディレクトリ: %s)",
        len(trajectories),
        directory_path,
    )
    return trajectories
