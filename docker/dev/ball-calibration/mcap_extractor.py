"""mcapファイルからボールキャリブレーション軌道データを抽出するモジュール.

C++ BallCalibrationDataExtractor の Python 移植版。
rosbags ライブラリを使用してROS2不要でmcapファイルを読み取る。
"""

from __future__ import annotations

import bisect
import logging
import math
from collections import deque
from pathlib import Path

from models import TrajectoryData

logger = logging.getLogger(__name__)

# 抽出設定（C++ ExtractorConfig のデフォルト値）
MIN_KICK_SPEED = 0.8  # 最小キック速度 [m/s]
MAX_KICK_SPEED = 30.0  # 最大キック速度 [m/s]
MAX_PRE_KICK_SPEED = 0.05  # キック前最大速度 [m/s]
REQUIRED_STATIONARY_FRAMES = 10  # 必要な静止フレーム数
MIN_TRAJECTORY_POINTS = 10  # 最小軌道点数

MIN_DT = 1e-5  # 最小時間間隔 [s]
MAX_DT = 0.2  # 最大時間間隔 [s]

DEFAULT_FIELD_LENGTH_HALF = 6.0  # [m]
DEFAULT_FIELD_WIDTH_HALF = 4.5  # [m]

# 軌道抽出パラメータ
_POSITION_THRESHOLD = 0.05  # 位置変化の閾値 [m]
_STATIONARY_DURATION = 0.3  # 静止判定継続時間 [s]
_TELEPORT_WINDOW = 0.1  # テレポート除外ウィンドウ [s]
_TELEPORT_SPEED = 0.1  # テレポート判定速度閾値 [m/s]

# ボール状態定数
_STATE_STOPPED = 0
_STATE_ROLLING = 1
_STATE_FLYING = 2

# typestore キャッシュ（bag パスごとに保持、最大10件）
_typestore_cache: dict = {}
_TYPESTORE_CACHE_MAX = 10

_MSGDEF_SEPARATOR = "=" * 80 + "\n"


def _parse_msgdef_types(msgtype: str, definition: str) -> dict:
    """bag の msgdef テキスト（マルチ型定義）から型 dict を構築する."""
    from rosbags.typesys import get_types_from_msg

    def normalize(name: str) -> str:
        """crane_msgs/BallInfo → crane_msgs/msg/BallInfo に変換."""
        parts = name.split("/")
        return f"{parts[0]}/msg/{parts[1]}" if len(parts) == 2 else name

    add_types: dict = {}
    parts = definition.split(_MSGDEF_SEPARATOR)

    # 最初のブロックがメイン型
    try:
        add_types.update(get_types_from_msg(parts[0], normalize(msgtype)))
    except Exception as e:
        logger.debug("型パーススキップ %s: %s", msgtype, e)

    # 残りは依存型（MSG: typename ヘッダ付き）
    for part in parts[1:]:
        if not part.startswith("MSG: "):
            continue
        nl = part.index("\n")
        sub_type = normalize(part[5:nl].strip())
        sub_def = part[nl + 1 :]
        try:
            add_types.update(get_types_from_msg(sub_def, sub_type))
        except Exception as e:
            logger.debug("型パーススキップ %s: %s", sub_type, e)

    return add_types


def _build_typestore(bag_path: Path, reader) -> object:
    """bag に埋め込まれた型定義から Typestore を構築してキャッシュする."""
    key = str(bag_path.resolve())
    if key in _typestore_cache:
        return _typestore_cache[key]

    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    add_types: dict = {}

    for conn in reader.connections:
        if conn.msgdef is None:
            continue
        _fmt, definition = conn.msgdef
        add_types.update(_parse_msgdef_types(conn.msgtype, definition))

    if add_types:
        typestore.register(add_types)
        logger.info("bag内の型定義を登録: %d 型", len(add_types))

    if len(_typestore_cache) >= _TYPESTORE_CACHE_MAX:
        _typestore_cache.pop(next(iter(_typestore_cache)))
    _typestore_cache[key] = typestore
    return typestore


def _extract_ball_and_command_data(bag_path: Path) -> tuple[list, list, float, float]:
    """mcapファイルからボールデータとロボットコマンドデータを抽出する.

    Returns:
        ball_data: list of (timestamp_ns, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, state, detected)
        command_data: list of (timestamp_ns, kick_power, chip_enable)
        field_length_half, field_width_half: フィールドサイズ [m]
    """
    from rosbags.rosbag2 import Reader

    ball_data: list[tuple] = []
    command_data: list[tuple] = []

    pos_history: deque[tuple[float, float]] = deque(maxlen=5)
    vel_history: deque[tuple[float, float]] = deque(maxlen=5)

    field_length_half = DEFAULT_FIELD_LENGTH_HALF
    field_width_half = DEFAULT_FIELD_WIDTH_HALF
    field_info_set = False

    try:
        with Reader(str(bag_path)) as reader:
            typestore = _build_typestore(bag_path, reader)

            wm_conns = [c for c in reader.connections if c.topic == "/world_model"]
            cmd_conns = [c for c in reader.connections if c.topic == "/robot_commands"]

            if not wm_conns:
                logger.warning("/world_model トピックが見つかりません")
            if not cmd_conns:
                logger.warning("/robot_commands トピックが見つかりません")

            for connection, timestamp_ns, rawdata in reader.messages(
                connections=wm_conns + cmd_conns
            ):
                if connection.topic == "/world_model":
                    try:
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        ball_info = msg.ball_info

                        if not (
                            ball_info.detected
                            or getattr(ball_info, "vision_detected", False)
                            or getattr(ball_info, "tracker_detected", False)
                        ):
                            continue

                        # フィールドサイズを初回取得
                        if not field_info_set:
                            fi = msg.field_info
                            if hasattr(fi, "x") and fi.x > 0.0 and fi.y > 0.0:
                                field_length_half = fi.x / 2.0
                                field_width_half = fi.y / 2.0
                                field_info_set = True

                        # Vision優先、なければ統合位置を使用（trackerフィールドはbagバージョン依存）
                        vision = getattr(ball_info, "vision", None)
                        if vision is not None and vision.stamp.sec != 0:
                            raw_x = float(vision.pos.x)
                            raw_y = float(vision.pos.y)
                            raw_z = float(vision.pos.z)
                        else:
                            raw_x = float(ball_info.position.x)
                            raw_y = float(ball_info.position.y)
                            raw_z = float(ball_info.position.z)

                        # 移動平均で位置ノイズを除去
                        all_x = [p[0] for p in pos_history] + [raw_x]
                        all_y = [p[1] for p in pos_history] + [raw_y]
                        smooth_x = sum(all_x) / len(all_x)
                        smooth_y = sum(all_y) / len(all_y)
                        pos_history.append((raw_x, raw_y))

                        vel_x, vel_y = 0.0, 0.0

                        if ball_data:
                            prev_ts, prev_sx, prev_sy = (
                                ball_data[-1][0],
                                ball_data[-1][1],
                                ball_data[-1][2],
                            )
                            dt = (timestamp_ns - prev_ts) * 1e-9

                            if MIN_DT <= dt <= MAX_DT:
                                raw_vx = (smooth_x - prev_sx) / dt
                                raw_vy = (smooth_y - prev_sy) / dt
                                raw_speed = math.hypot(raw_vx, raw_vy)

                                if raw_speed <= MAX_KICK_SPEED:
                                    # 急激な加速度スパイクを除去
                                    if vel_history:
                                        prev_vx, prev_vy = vel_history[-1]
                                        prev_speed = math.hypot(prev_vx, prev_vy)
                                        if (
                                            prev_speed > 0.01
                                            and abs(raw_speed - prev_speed) / dt > 500.0
                                        ):
                                            raw_vx, raw_vy = prev_vx, prev_vy
                                    vel_x, vel_y = raw_vx, raw_vy
                                elif vel_history:
                                    vel_x, vel_y = vel_history[-1]
                            elif dt > MAX_DT:
                                vel_x, vel_y = 0.0, 0.0
                            elif vel_history:
                                vel_x, vel_y = vel_history[-1]

                        vel_history.append((vel_x, vel_y))

                        speed = math.hypot(vel_x, vel_y)
                        if speed < 0.05:
                            state = _STATE_STOPPED
                        elif raw_z > 0.02:
                            state = _STATE_FLYING
                        else:
                            state = _STATE_ROLLING

                        ball_data.append(
                            (
                                timestamp_ns,
                                smooth_x,
                                smooth_y,
                                raw_z,
                                vel_x,
                                vel_y,
                                0.0,
                                state,
                                True,
                            )
                        )

                    except Exception as e:
                        logger.debug("world_model デシリアライズエラー: %s", e)

                elif connection.topic == "/robot_commands":
                    try:
                        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                        for cmd in msg.robot_commands:
                            if float(cmd.kick_power) > 0.0:
                                command_data.append(
                                    (
                                        timestamp_ns,
                                        float(cmd.kick_power),
                                        bool(cmd.chip_enable),
                                    )
                                )
                    except Exception as e:
                        logger.debug("robot_commands デシリアライズエラー: %s", e)

    except Exception as e:
        logger.error("mcapファイル読み取りエラー: %s", e)
        raise

    logger.info(
        "データ抽出完了: ボールデータ %d 点, ロボットコマンド %d 件",
        len(ball_data),
        len(command_data),
    )
    return ball_data, command_data, field_length_half, field_width_half


def _detect_kick_events(ball_data: list[tuple]) -> list[tuple[int, float, float]]:
    """キックイベントを検出する（スライディングウィンドウ方式）.

    Returns:
        list of (timestamp_ns, pos_x, pos_y)
    """
    kick_events: list[tuple[int, float, float]] = []

    if len(ball_data) < REQUIRED_STATIONARY_FRAMES + 1:
        return kick_events

    # 連続静止フレーム数をスライディングウィンドウで追跡
    stationary_count = 0
    for entry in ball_data:
        ts, sx, sy, _, vx, vy, _, _, _ = entry
        speed = math.hypot(vx, vy)

        if speed < MAX_PRE_KICK_SPEED:
            stationary_count += 1
        else:
            # 十分な静止フレームの後に速度が MIN_KICK_SPEED を超えたらキックイベント
            if (
                stationary_count >= REQUIRED_STATIONARY_FRAMES
                and speed > MIN_KICK_SPEED
            ):
                kick_events.append((ts, sx, sy))
            stationary_count = 0

    logger.info("キックイベント検出: %d 件", len(kick_events))
    return kick_events


def _is_field_inside(
    x: float,
    y: float,
    field_length_half: float,
    field_width_half: float,
    offset: float = 0.0,
) -> bool:
    return abs(x) <= (field_length_half + offset) and abs(y) <= (
        field_width_half + offset
    )


def _extract_trajectory(
    ball_data: list[tuple],
    timestamps: list[int],
    kick_ts: int,
    field_length_half: float,
    field_width_half: float,
) -> tuple[list[tuple], bool]:
    """キックイベント後のボール軌道を抽出する.

    Returns:
        (trajectory_data, has_movement)
        trajectory_data: list of (timestamp_ns, pos_x, pos_y, speed, state)
    """
    start_idx = bisect.bisect_right(timestamps, kick_ts)
    if start_idx >= len(ball_data):
        return [], False

    trajectory: list[tuple] = []
    has_movement = False
    stationary_start_ts = -1.0
    last_pos_x, last_pos_y = ball_data[start_idx][1], ball_data[start_idx][2]
    outside_field = False
    teleport_count = 0
    teleport_disabled = False

    for i in range(start_idx, len(ball_data)):
        ts, sx, sy, _, vx, vy, _, state, _ = ball_data[i]
        elapsed = (ts - kick_ts) * 1e-9
        speed = math.hypot(vx, vy)
        traj_state = state

        if not outside_field and not _is_field_inside(
            sx, sy, field_length_half, field_width_half, -0.2
        ):
            outside_field = True

        if outside_field:
            traj_state = 3  # INVALID

        # テレポートキック除外（キック直後に速度が閾値以下になった場合）
        if (
            not teleport_disabled
            and elapsed <= _TELEPORT_WINDOW
            and speed <= _TELEPORT_SPEED
        ):
            teleport_count += 1
            if teleport_count >= 5:
                teleport_disabled = True
            break

        trajectory.append((ts, sx, sy, speed, traj_state))

        position_change = math.hypot(sx - last_pos_x, sy - last_pos_y)
        if position_change > _POSITION_THRESHOLD:
            has_movement = True
            stationary_start_ts = -1.0
            last_pos_x, last_pos_y = sx, sy
        elif elapsed > 0.5:
            if stationary_start_ts < 0:
                stationary_start_ts = elapsed
            elif elapsed - stationary_start_ts > _STATIONARY_DURATION and has_movement:
                break

    # 停止直前の微動データを末尾からトリム（停止点から5cm以内の点を除去）
    if has_movement and trajectory:
        stop_x, stop_y = trajectory[-1][1], trajectory[-1][2]
        cutoff = len(trajectory)
        for i in range(len(trajectory) - 1, -1, -1):
            if math.hypot(trajectory[i][1] - stop_x, trajectory[i][2] - stop_y) > 0.05:
                cutoff = i + 1
                break
        trajectory = trajectory[: max(cutoff, 3)]

    return trajectory, has_movement


def _trajectory_to_data(
    event_id: int,
    kick_power: float,
    is_chip_kick: bool,
    raw_trajectory: list[tuple],
) -> TrajectoryData | None:
    """生軌道データ（ROLLING状態のみ）を TrajectoryData 形式に変換する."""
    rolling_pts = [
        (ts, x, y)
        for ts, x, y, speed, state in raw_trajectory
        if state == _STATE_ROLLING
    ]

    if not rolling_pts:
        return None

    start_ts = rolling_pts[0][0]
    time_points = [(ts - start_ts) * 1e-9 for ts, x, y in rolling_pts]
    positions_x = [x for _, x, _ in rolling_pts]
    positions_y = [y for _, _, y in rolling_pts]

    velocities: list[float] = []
    n = len(time_points)
    for i in range(n):
        if i == 0:
            if n > 1:
                dx = positions_x[1] - positions_x[0]
                dy = positions_y[1] - positions_y[0]
                dt = time_points[1] - time_points[0]
                v = math.hypot(dx, dy) / dt if dt > 0 else 0.0
            else:
                v = 0.0
        else:
            dx = positions_x[i] - positions_x[i - 1]
            dy = positions_y[i] - positions_y[i - 1]
            dt = time_points[i] - time_points[i - 1]
            v = math.hypot(dx, dy) / dt if dt > 0 else 0.0
        velocities.append(v)

    return TrajectoryData(
        event_id=event_id,
        kick_power=kick_power,
        is_chip_kick=is_chip_kick,
        time_points=time_points,
        positions_x=positions_x,
        positions_y=positions_y,
        velocities=velocities,
    )


def extract_trajectories_from_mcap(mcap_path: Path) -> list[TrajectoryData]:
    """mcapファイルからキャリブレーション用軌道データを抽出する."""
    logger.info("mcap抽出開始: %s", mcap_path)

    ball_data, command_data, field_length_half, field_width_half = (
        _extract_ball_and_command_data(mcap_path)
    )

    if not ball_data:
        logger.warning("ボールデータが抽出できませんでした")
        return []

    kick_events = _detect_kick_events(ball_data)
    if not kick_events:
        logger.warning("キックイベントが検出されませんでした")
        return []

    # bisect 用にタイムスタンプリストを事前生成
    timestamps = [e[0] for e in ball_data]
    cmd_timestamps = [c[0] for c in command_data]
    _MATCH_WINDOW_NS = 5 * 10**9  # 5秒

    trajectories: list[TrajectoryData] = []
    event_id = 0

    for kick_ts, _, _ in kick_events:
        # bisect で5秒ウィンドウ内のコマンドを絞り込んで最近接を探す
        lo = bisect.bisect_left(cmd_timestamps, kick_ts - _MATCH_WINDOW_NS)
        hi = bisect.bisect_right(cmd_timestamps, kick_ts + _MATCH_WINDOW_NS)
        best_cmd: tuple[float, bool] | None = None
        best_dt = float("inf")
        for cmd_ts, kick_power, chip_enable in command_data[lo:hi]:
            dt = abs((cmd_ts - kick_ts) * 1e-9)
            if dt < best_dt:
                best_dt = dt
                best_cmd = (kick_power, chip_enable)

        if best_cmd is None:
            continue

        kick_power, is_chip_kick = best_cmd

        raw_traj, has_movement = _extract_trajectory(
            ball_data, timestamps, kick_ts, field_length_half, field_width_half
        )

        if not has_movement or len(raw_traj) < MIN_TRAJECTORY_POINTS:
            continue

        traj = _trajectory_to_data(event_id, kick_power, is_chip_kick, raw_traj)
        if traj is not None and len(traj.time_points) >= MIN_TRAJECTORY_POINTS:
            trajectories.append(traj)
            event_id += 1

    logger.info("軌道抽出完了: %d 件", len(trajectories))
    return trajectories
