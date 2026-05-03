"""trackサブコマンド: ロボット/ボールの時系列追跡を表示する."""

from __future__ import annotations

import json

from .formatters import fmt_pos, fmt_t
from .models import LogData


def run_track(
    data: LogData,
    *,
    robot_id: int | None = None,
    team: str | None = None,
    ball: bool = False,
    interval: float = 1.0,
    time_range: tuple[float, float] | None = None,
    fmt: str = "text",
) -> None:
    start = time_range[0] if time_range else 0.0
    end = time_range[1] if time_range else data.info.duration_sec

    if ball:
        _track_ball(data, start, end, interval, fmt)
    elif robot_id is not None:
        _track_robot(data, robot_id, team, start, end, interval, fmt)
    else:
        print("--ball または --robot <id> を指定してください")


def _track_ball(
    data: LogData, start: float, end: float, interval: float, fmt: str
) -> None:
    timeline = [b for b in data.ball_timeline if start <= b.t <= end]
    if not timeline:
        print("ボールデータなし")
        return

    sampled = _sample_by_interval(timeline, interval, key=lambda b: b.t)

    if fmt == "json":
        output = [
            {"t": b.t, "x": b.x, "y": b.y, "z": b.z, "speed": b.speed, "state": b.state}
            for b in sampled
        ]
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return

    for b in sampled:
        kick_mark = " [KICK]" if b.has_kicked_ball else ""
        print(
            f"  {fmt_t(b.t)}: pos={fmt_pos(b.x, b.y)} z={b.z:.2f}m "
            f"speed={b.speed:.2f}m/s  {b.state}{kick_mark}"
        )


def _track_robot(
    data: LogData,
    robot_id: int,
    team: str | None,
    start: float,
    end: float,
    interval: float,
    fmt: str,
) -> None:
    keys = [(t, rid) for (t, rid) in data.robot_timeline if rid == robot_id]
    if team:
        keys = [(t, rid) for (t, rid) in keys if t == team]
    if not keys:
        print(f"ロボットID {robot_id} のデータが見つかりません（team={team or 'any'}）")
        return

    for key in sorted(keys):
        t_label, rid = key
        snapshots = [r for r in data.robot_timeline[key] if start <= r.t <= end]
        if not snapshots:
            continue
        sampled = _sample_by_interval(snapshots, interval, key=lambda r: r.t)
        print(f"\n--- {t_label.upper()} #{rid} ---")

        if fmt == "json":
            output = [
                {
                    "t": r.t,
                    "x": r.x,
                    "y": r.y,
                    "orientation": r.orientation,
                    "speed": r.speed,
                    "team": r.team,
                    "robot_id": r.robot_id,
                }
                for r in sampled
            ]
            print(json.dumps(output, ensure_ascii=False, indent=2))
            continue

        for r in sampled:
            print(
                f"  {fmt_t(r.t)}: pos={fmt_pos(r.x, r.y)} "
                f"ori={r.orientation:.2f}rad  speed={r.speed:.2f}m/s"
            )


def _sample_by_interval(items: list, interval: float, key) -> list:
    """intervalごとに1件サンプリングする."""
    if not items:
        return []
    result = [items[0]]
    last_t = key(items[0])
    for item in items[1:]:
        if key(item) - last_t >= interval:
            result.append(item)
            last_t = key(item)
    if result[-1] is not items[-1]:
        result.append(items[-1])
    return result
