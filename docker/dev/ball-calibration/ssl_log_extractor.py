"""SSL公式ログ (.log.gz) からボールキャリブレーション軌道データを抽出するモジュール."""

from __future__ import annotations

import bisect
import gzip
import logging
import math
import struct
import sys
from pathlib import Path

from models import TrajectoryData

logger = logging.getLogger(__name__)

# ssl_proto/ を sys.path に追加（protoc 生成ファイルのインポート用）
_SSL_PROTO_DIR = Path(__file__).parent / "ssl_proto"
if str(_SSL_PROTO_DIR) not in sys.path:
    sys.path.insert(0, str(_SSL_PROTO_DIR))

# メッセージ ID 定数（pkg/persistence/message.go より）
_MSG_VISION_2014 = 4
_MSG_TRACKER_2020 = 5
_MSG_INDEX_2021 = 6

# ファイルヘッダ
_LOG_MAGIC = b"SSL_LOG_FILE"
_MSG_HEADER_FMT = ">qii"  # int64 timestamp_ns, int32 msg_id, int32 length
_MSG_HEADER_SIZE = 16  # 8 + 4 + 4

# ボール状態定数
_STATE_STOPPED = 0
_STATE_ROLLING = 1
_STATE_FLYING = 2

# 軌道抽出パラメータ
_POSITION_THRESHOLD = 0.05  # [m] 位置変化の閾値
_STATIONARY_DURATION = 0.3  # [s] 静止判定継続時間
_TELEPORT_WINDOW = 0.1  # [s] テレポート除外ウィンドウ
_TELEPORT_SPEED = 0.1  # [m/s] テレポート判定速度閾値

MIN_TRAJECTORY_POINTS = 10

DEFAULT_FIELD_LENGTH_HALF = 6.0  # [m]
DEFAULT_FIELD_WIDTH_HALF = 4.5  # [m]


def _iter_messages(path: Path):
    """ログファイルを走査し (timestamp_ns, msg_id, payload) を yield する."""
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rb") as f:
        magic = f.read(12)
        if magic != _LOG_MAGIC:
            raise ValueError(f"SSL ログのマジックが不正です: {magic!r}")
        (version,) = struct.unpack(">i", f.read(4))
        if version != 1:
            raise ValueError(f"未対応のバージョン: {version}")

        while True:
            hdr = f.read(_MSG_HEADER_SIZE)
            if not hdr:
                break
            if len(hdr) < _MSG_HEADER_SIZE:
                logger.warning("ヘッダ読み取り不足で終了")
                break
            ts_ns, msg_id, length = struct.unpack(_MSG_HEADER_FMT, hdr)
            payload = f.read(length)
            if len(payload) < length:
                logger.warning("ペイロード読み取り不足で終了")
                break
            if msg_id == _MSG_INDEX_2021:
                break
            yield ts_ns, msg_id, payload


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
    """キックイベント後のボール軌道を抽出する (mcap_extractor から流用)."""
    start_idx = bisect.bisect_right(timestamps, kick_ts)
    if start_idx >= len(ball_data):
        return [], False

    trajectory: list[tuple] = []
    has_movement = False
    stationary_start_ts = -1.0
    last_pos_x, last_pos_y = ball_data[start_idx][1], ball_data[start_idx][2]
    outside_field = False
    teleport_count = 0

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

        if elapsed <= _TELEPORT_WINDOW and speed <= _TELEPORT_SPEED:
            teleport_count += 1
            if teleport_count >= 5:
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

        if has_movement and elapsed > 0.4 and len(trajectory) >= 8:
            window_pts = [
                p for p in trajectory if elapsed - (p[0] - kick_ts) * 1e-9 <= 0.4
            ]
            if len(window_pts) >= 4:
                w_spd = [p[3] for p in window_pts]
                w_t = [(p[0] - window_pts[0][0]) * 1e-9 for p in window_pts]
                mean_w = sum(w_spd) / len(w_spd)
                ss_tot = sum((v - mean_w) ** 2 for v in w_spd)
                if ss_tot > 1e-6 and mean_w < 0.2:
                    a_w = sum(
                        (t - sum(w_t) / len(w_t)) * (v - mean_w)
                        for t, v in zip(w_t, w_spd)
                    ) / sum((t - sum(w_t) / len(w_t)) ** 2 for t in w_t)
                    b_w = mean_w - a_w * (sum(w_t) / len(w_t))
                    ss_res = sum((v - (a_w * t + b_w)) ** 2 for t, v in zip(w_t, w_spd))
                    r2_window = 1.0 - ss_res / ss_tot
                    if r2_window < 0.3:
                        break

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
    raw_trajectory: list[tuple],
) -> TrajectoryData | None:
    """生軌道データ (ROLLING 状態のみ) を TrajectoryData 形式に変換する."""
    rolling_pts = [
        (ts, x, y, speed)
        for ts, x, y, speed, state in raw_trajectory
        if state == _STATE_ROLLING
    ]

    if not rolling_pts:
        return None

    start_ts = rolling_pts[0][0]
    return TrajectoryData(
        event_id=event_id,
        time_points=[(ts - start_ts) * 1e-9 for ts, _, _, _ in rolling_pts],
        positions_x=[x for _, x, _, _ in rolling_pts],
        positions_y=[y for _, _, y, _ in rolling_pts],
        velocities=[speed for _, _, _, speed in rolling_pts],
    )


def extract_ball_timeline_and_trajectories(
    path: Path | str,
) -> tuple[list[TrajectoryData], list[tuple]]:
    """SSL公式ログから軌道と生のボールタイムラインを返す.

    Returns:
        (trajectories, ball_data)
        ball_data: (timestamp_ns, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, state, detected)
    """
    path = Path(path)
    logger.info("SSL ログ抽出開始: %s", path)

    try:
        from ssl_vision_wrapper_pb2 import SSL_WrapperPacket
        from ssl_vision_wrapper_tracked_pb2 import TrackerWrapperPacket
    except ImportError as e:
        raise RuntimeError(
            f"protobuf モジュールが見つかりません。sync_proto.sh を実行してから Docker を"
            f"ビルドしてください: {e}"
        ) from e

    ball_data: list[tuple] = []
    kick_events: list[tuple] = []  # (ts_ns, pos_x, pos_y)

    field_length_half = DEFAULT_FIELD_LENGTH_HALF
    field_width_half = DEFAULT_FIELD_WIDTH_HALF
    geometry_loaded = False

    # 最初に見たトラッカー UUID を primary として固定（複数ソース対策）
    primary_uuid: str | None = None
    prev_had_kicked_ball = False

    for ts_ns, msg_id, payload in _iter_messages(path):
        if msg_id == _MSG_VISION_2014 and not geometry_loaded:
            pkt = SSL_WrapperPacket()
            pkt.ParseFromString(payload)
            if pkt.HasField("geometry") and pkt.geometry.HasField("field"):
                field = pkt.geometry.field
                # field_length/width は mm 単位の int32
                field_length_half = field.field_length / 2000.0
                field_width_half = field.field_width / 2000.0
                geometry_loaded = True
                logger.info(
                    "フィールドサイズ取得: %.1f x %.1f m",
                    field_length_half * 2,
                    field_width_half * 2,
                )

        elif msg_id == _MSG_TRACKER_2020:
            pkt = TrackerWrapperPacket()
            pkt.ParseFromString(payload)

            if primary_uuid is None:
                primary_uuid = pkt.uuid
            elif pkt.uuid != primary_uuid:
                continue

            if not pkt.HasField("tracked_frame"):
                continue
            frame = pkt.tracked_frame

            frame_ts_ns = int(frame.timestamp * 1e9)

            # ボールデータ
            if frame.balls:
                ball = frame.balls[0]
                pos = ball.pos
                if ball.HasField("vel"):
                    vel = ball.vel
                    vx, vy, vz = vel.x, vel.y, vel.z
                else:
                    vx, vy, vz = 0.0, 0.0, 0.0

                speed = math.hypot(vx, vy)
                if speed < 0.05:
                    state = _STATE_STOPPED
                elif pos.z > 0.02:
                    state = _STATE_FLYING
                else:
                    state = _STATE_ROLLING

                ball_data.append(
                    (frame_ts_ns, pos.x, pos.y, pos.z, vx, vy, vz, state, True)
                )

            # キック検出: kicked_ball の無→有 遷移でイベント発火
            has_kicked_ball = frame.HasField("kicked_ball")
            if has_kicked_ball and not prev_had_kicked_ball:
                kb = frame.kicked_ball
                kick_events.append((frame_ts_ns, kb.pos.x, kb.pos.y))
                logger.debug(
                    "キックイベント検出: t=%.3f, pos=(%.2f, %.2f), vel=(%.2f, %.2f, %.2f)",
                    frame.timestamp,
                    kb.pos.x,
                    kb.pos.y,
                    kb.vel.x,
                    kb.vel.y,
                    kb.vel.z,
                )
            prev_had_kicked_ball = has_kicked_ball

    if not ball_data:
        logger.warning("ボールデータが抽出できませんでした")
        return [], []

    logger.info(
        "抽出完了: ball=%d点, kicks=%d件",
        len(ball_data),
        len(kick_events),
    )

    # キックイベントごとに軌道を抽出
    timestamps = [d[0] for d in ball_data]
    trajectories: list[TrajectoryData] = []
    event_id = 0

    for kick_ts, _, _ in kick_events:
        raw_traj, has_movement = _extract_trajectory(
            ball_data, timestamps, kick_ts, field_length_half, field_width_half
        )
        if not has_movement or len(raw_traj) < MIN_TRAJECTORY_POINTS:
            continue

        traj = _trajectory_to_data(event_id, raw_traj)
        if traj is not None and len(traj.time_points) >= MIN_TRAJECTORY_POINTS:
            trajectories.append(traj)
            event_id += 1

    logger.info("軌道抽出完了: %d 件", len(trajectories))
    return trajectories, ball_data
