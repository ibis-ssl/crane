"""SSL公式ログ解析用データモデル."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class LogInfo:
    path: str
    start_ts_ns: int
    end_ts_ns: int
    duration_sec: float
    msg_counts: dict[int, int]
    yellow_team_name: str
    blue_team_name: str
    field_length_m: float
    field_width_m: float


@dataclass
class RefereeState:
    ts_ns: int
    t: float
    stage: str
    command: str
    command_counter: int
    yellow_score: int
    blue_score: int
    yellow_name: str
    blue_name: str
    yellow_yellow_cards: int
    blue_yellow_cards: int
    yellow_red_cards: int
    blue_red_cards: int
    game_event_types: list[str] = field(default_factory=list)


@dataclass
class BallSnapshot:
    ts_ns: int
    t: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    speed: float
    state: str
    has_kicked_ball: bool


@dataclass
class RobotSnapshot:
    ts_ns: int
    t: float
    team: str
    robot_id: int
    x: float
    y: float
    orientation: float
    vx: float
    vy: float
    speed: float
    visibility: float


@dataclass
class LogEvent:
    ts_ns: int
    t: float
    event_type: str
    description: str
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class LogData:
    info: LogInfo
    referee_states: list[RefereeState]
    ball_timeline: list[BallSnapshot]
    robot_timeline: dict[tuple[str, int], list[RobotSnapshot]]

    def time_slice(self, start: float, end: float) -> LogData:
        """指定時間範囲のデータを返す."""
        ref = [r for r in self.referee_states if start <= r.t <= end]
        ball = [b for b in self.ball_timeline if start <= b.t <= end]
        robots = {
            k: [r for r in v if start <= r.t <= end]
            for k, v in self.robot_timeline.items()
        }
        return LogData(self.info, ref, ball, robots)
