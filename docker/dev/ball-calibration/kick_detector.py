"""汎用キックイベント検出モジュール.

事前静止を必要としない速度ジャンプベースの検出。
試合/練習データからでも副産物的なキック軌道を取り出せる。
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

from models import KickDetectorConfig

logger = logging.getLogger(__name__)


@dataclass
class KickEvent:
    """検出されたキックイベント."""

    ts_ns: int
    pos_x: float
    pos_y: float
    v_jump: float
    owner_robot_id: int | None
    confidence: float


def _nearest_robot(
    robot_data: list[tuple],
    ts_ns: int,
    ball_x: float,
    ball_y: float,
    radius: float,
    window_ns: int = 200_000_000,  # 200ms
) -> int | None:
    """指定タイムスタンプ周辺でボールに最も近いロボットIDを返す."""
    best_id = None
    best_dist = radius

    for rts, _team, rid, rx, ry in robot_data:
        if abs(rts - ts_ns) > window_ns:
            continue
        dist = math.hypot(rx - ball_x, ry - ball_y)
        if dist < best_dist:
            best_dist = dist
            best_id = rid

    return best_id


def detect_kick_events(
    ball_data: list[tuple],
    robot_data: list[tuple] | None,
    cfg: KickDetectorConfig | None = None,
) -> list[KickEvent]:
    """ボールデータからキックイベントを検出する.

    Args:
        ball_data: [(ts_ns, x, y, z, vx, vy, vz, state, detected), ...]
        robot_data: [(ts_ns, team, id, x, y), ...] or None（所有者判定不要時）
        cfg: 検出設定（None の場合デフォルト値）

    Returns:
        検出されたキックイベントのリスト
    """
    if cfg is None:
        cfg = KickDetectorConfig()

    if len(ball_data) < 2:
        return []

    events: list[KickEvent] = []
    cooldown_ns = int(cfg.cooldown_s * 1e9)
    last_event_ts: int = -cooldown_ns * 2  # 初回は必ず検出可能

    for i in range(1, len(ball_data)):
        ts, sx, sy, _, vx, vy, _, _, _ = ball_data[i]
        prev_ts, _, _, _, prev_vx, prev_vy, _, _, _ = ball_data[i - 1]

        dt = (ts - prev_ts) * 1e-9
        if dt <= 0:
            continue

        speed = math.hypot(vx, vy)
        prev_speed = math.hypot(prev_vx, prev_vy)

        delta_v = speed - prev_speed

        # 速度ジャンプ: 前フレームより大幅に増加 かつ 最小キック速度を超えた
        if delta_v < cfg.min_delta_v:
            continue
        if speed < cfg.min_kick_speed or speed > cfg.max_kick_speed:
            continue

        # 加速度チェック
        accel = delta_v / dt
        if accel < cfg.min_accel:
            continue

        # クールダウン: 前イベントから十分時間が経過していないと除外
        if ts - last_event_ts < cooldown_ns:
            continue

        # 所有者判定（robot_data が提供されている場合）
        owner_id: int | None = None
        if robot_data:
            owner_id = _nearest_robot(robot_data, ts, sx, sy, cfg.ownership_radius)

        confidence = min(1.0, delta_v / cfg.min_kick_speed)
        events.append(
            KickEvent(
                ts_ns=ts,
                pos_x=sx,
                pos_y=sy,
                v_jump=delta_v,
                owner_robot_id=owner_id,
                confidence=confidence,
            )
        )
        last_event_ts = ts

    logger.info(
        "キックイベント検出（汎用）: %d 件（Δv≥%.1f, accel≥%.1f m/s², cooldown=%.1fs）",
        len(events),
        cfg.min_delta_v,
        cfg.min_accel,
        cfg.cooldown_s,
    )
    return events
