"""eventsサブコマンド: ゲームイベントを検出・表示する."""

from __future__ import annotations

import json
import math

from .formatters import fmt_t
from .models import LogData, LogEvent

_FOUL_TYPES = {
    "BOT_KICKED_BALL_TOO_FAST",
    "BOT_CRASH_UNIQUE",
    "BOT_CRASH_DRAWN",
    "BOT_PUSHED_BOT",
    "BOT_HELD_BALL_DELIBERATELY",
    "BOT_DRIBBLED_BALL_TOO_FAR",
    "BOT_TIPPED_OVER",
    "BOT_TOO_FAST_IN_STOP",
    "ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA",
    "ATTACKER_IN_DEFENSE_AREA",
    "DEFENDER_IN_DEFENSE_AREA",
    "DEFENDER_IN_DEFENSE_AREA_PARTIALLY",
    "DEFENDER_TOO_CLOSE_TO_KICK_POINT",
    "ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA",
    "KEEPER_HELD_BALL",
    "BOUNDARY_CROSSING",
    "ATTACKER_DOUBLE_TOUCHED_BALL",
    "BOT_INTERFERED_PLACEMENT",
    "UNSPORTING_BEHAVIOR_MINOR",
    "UNSPORTING_BEHAVIOR_MAJOR",
}

_PLACEMENT_TYPES = {
    "PLACEMENT_SUCCEEDED",
    "PLACEMENT_FAILED",
    "MULTIPLE_PLACEMENT_FAILURES",
}


def run_events(
    data: LogData,
    types: list[str] | None = None,
    fmt: str = "text",
) -> None:
    events = _collect_events(
        data, types or ["kick", "referee", "foul", "goal", "placement", "card"]
    )

    if fmt == "json":
        output = [
            {
                "t": e.t,
                "type": e.event_type,
                "description": e.description,
                **{
                    k: (
                        v
                        if not isinstance(v, float)
                        or (math.isfinite(v) and abs(v) < 1e308)
                        else None
                    )
                    for k, v in e.details.items()
                },
            }
            for e in events
        ]
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return

    for e in events:
        print(f"  {fmt_t(e.t)}: [{e.event_type}] {e.description}")


def _collect_events(data: LogData, types: list[str]) -> list[LogEvent]:
    events: list[LogEvent] = []
    type_set = set(types)

    if "kick" in type_set:
        events.extend(_detect_kicks(data))
    if "referee" in type_set:
        events.extend(_detect_referee_changes(data))
    if "foul" in type_set:
        events.extend(_detect_game_events(data, _FOUL_TYPES, "foul"))
    if "goal" in type_set:
        events.extend(_detect_goals(data))
    if "placement" in type_set:
        events.extend(_detect_game_events(data, _PLACEMENT_TYPES, "placement"))
    if "card" in type_set:
        events.extend(_detect_cards(data))

    events.sort(key=lambda e: e.ts_ns)
    return events


def _detect_kicks(data: LogData) -> list[LogEvent]:
    result = []
    timeline = data.ball_timeline
    for i in range(1, len(timeline)):
        if timeline[i].has_kicked_ball and not timeline[i - 1].has_kicked_ball:
            b = timeline[i]
            result.append(
                LogEvent(
                    ts_ns=b.ts_ns,
                    t=b.t,
                    event_type="kick",
                    description=f"kick: pos=({b.x:.2f},{b.y:.2f}) speed={b.speed:.2f}m/s",
                    details={"x": b.x, "y": b.y, "speed": b.speed},
                )
            )
    return result


def _detect_referee_changes(data: LogData) -> list[LogEvent]:
    result = []
    prev_cmd = None
    for rs in data.referee_states:
        if rs.command != prev_cmd:
            result.append(
                LogEvent(
                    ts_ns=rs.ts_ns,
                    t=rs.t,
                    event_type="referee",
                    description=f"referee: {prev_cmd or '(start)'} -> {rs.command} ({rs.stage})",
                    details={"command": rs.command, "stage": rs.stage},
                )
            )
            prev_cmd = rs.command
    return result


def _detect_goals(data: LogData) -> list[LogEvent]:
    result = []
    for i in range(1, len(data.referee_states)):
        prev = data.referee_states[i - 1]
        cur = data.referee_states[i]
        if cur.yellow_score > prev.yellow_score:
            result.append(
                LogEvent(
                    ts_ns=cur.ts_ns,
                    t=cur.t,
                    event_type="goal",
                    description=f"GOAL: Yellow score {prev.yellow_score} -> {cur.yellow_score}",
                    details={"team": "yellow", "score": cur.yellow_score},
                )
            )
        if cur.blue_score > prev.blue_score:
            result.append(
                LogEvent(
                    ts_ns=cur.ts_ns,
                    t=cur.t,
                    event_type="goal",
                    description=f"GOAL: Blue score {prev.blue_score} -> {cur.blue_score}",
                    details={"team": "blue", "score": cur.blue_score},
                )
            )
    return result


def _detect_game_events(
    data: LogData, ge_types: set[str], event_type: str
) -> list[LogEvent]:
    result = []
    for rs in data.referee_states:
        for et in rs.game_event_types:
            if et in ge_types:
                result.append(
                    LogEvent(
                        ts_ns=rs.ts_ns,
                        t=rs.t,
                        event_type=event_type,
                        description=f"{et}",
                        details={"game_event_type": et},
                    )
                )
    return result


def _detect_cards(data: LogData) -> list[LogEvent]:
    result = []
    for i in range(1, len(data.referee_states)):
        prev = data.referee_states[i - 1]
        cur = data.referee_states[i]
        if cur.yellow_yellow_cards > prev.yellow_yellow_cards:
            result.append(
                LogEvent(
                    ts_ns=cur.ts_ns,
                    t=cur.t,
                    event_type="card",
                    description=f"YELLOW CARD -> Yellow team (total={cur.yellow_yellow_cards})",
                    details={"team": "yellow", "card": "yellow"},
                )
            )
        if cur.blue_yellow_cards > prev.blue_yellow_cards:
            result.append(
                LogEvent(
                    ts_ns=cur.ts_ns,
                    t=cur.t,
                    event_type="card",
                    description=f"YELLOW CARD -> Blue team (total={cur.blue_yellow_cards})",
                    details={"team": "blue", "card": "yellow"},
                )
            )
        if cur.yellow_red_cards > prev.yellow_red_cards:
            result.append(
                LogEvent(
                    ts_ns=cur.ts_ns,
                    t=cur.t,
                    event_type="card",
                    description=f"RED CARD -> Yellow team (total={cur.yellow_red_cards})",
                    details={"team": "yellow", "card": "red"},
                )
            )
        if cur.blue_red_cards > prev.blue_red_cards:
            result.append(
                LogEvent(
                    ts_ns=cur.ts_ns,
                    t=cur.t,
                    event_type="card",
                    description=f"RED CARD -> Blue team (total={cur.blue_red_cards})",
                    details={"team": "blue", "card": "red"},
                )
            )
    return result
