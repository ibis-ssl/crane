"""SSL公式ログ (.log.gz) 読み取りモジュール."""

from __future__ import annotations

import gzip
import math
import struct
from pathlib import Path
from typing import Iterator

from .models import BallSnapshot, LogData, LogInfo, RefereeState, RobotSnapshot
from .proto_loader import load_proto_modules

_MSG_REFEREE_2013 = 3
_MSG_VISION_2014 = 4
_MSG_TRACKER_2020 = 5
_MSG_INDEX_2021 = 6

_LOG_MAGIC = b"SSL_LOG_FILE"
_MSG_HEADER_FMT = ">qii"
_MSG_HEADER_SIZE = 16

_DEFAULT_FIELD_LENGTH_HALF = 6.0
_DEFAULT_FIELD_WIDTH_HALF = 4.5

_STAGE_NAMES = {
    0: "NORMAL_FIRST_HALF_PRE",
    1: "NORMAL_FIRST_HALF",
    2: "NORMAL_HALF_TIME",
    3: "NORMAL_SECOND_HALF_PRE",
    4: "NORMAL_SECOND_HALF",
    5: "EXTRA_TIME_BREAK",
    6: "EXTRA_FIRST_HALF_PRE",
    7: "EXTRA_FIRST_HALF",
    8: "EXTRA_HALF_TIME",
    9: "EXTRA_SECOND_HALF_PRE",
    10: "EXTRA_SECOND_HALF",
    11: "PENALTY_SHOOTOUT_BREAK",
    12: "PENALTY_SHOOTOUT",
    13: "POST_GAME",
}

_COMMAND_NAMES = {
    0: "HALT",
    1: "STOP",
    2: "NORMAL_START",
    3: "FORCE_START",
    4: "PREPARE_KICKOFF_YELLOW",
    5: "PREPARE_KICKOFF_BLUE",
    6: "PREPARE_PENALTY_YELLOW",
    7: "PREPARE_PENALTY_BLUE",
    8: "DIRECT_FREE_YELLOW",
    9: "DIRECT_FREE_BLUE",
    16: "TIMEOUT_YELLOW",
    17: "TIMEOUT_BLUE",
    20: "BALL_PLACEMENT_YELLOW",
    21: "BALL_PLACEMENT_BLUE",
}


def iter_messages(path: Path) -> Iterator[tuple[int, int, bytes]]:
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
            if not hdr or len(hdr) < _MSG_HEADER_SIZE:
                break
            ts_ns, msg_id, length = struct.unpack(_MSG_HEADER_FMT, hdr)
            if msg_id == _MSG_INDEX_2021:
                break
            if length <= 0 or length > 10_000_000:
                break
            payload = f.read(length)
            if len(payload) < length:
                break
            yield ts_ns, msg_id, payload


def load_log(path: Path | str) -> LogData:
    """SSL公式ログを全解析して LogData を返す."""
    path = Path(path)
    SSL_WrapperPacket, TrackerWrapperPacket, Referee = load_proto_modules()

    msg_counts: dict[int, int] = {}
    referee_states: list[RefereeState] = []
    ball_timeline: list[BallSnapshot] = []
    robot_timeline: dict[tuple[str, int], list[RobotSnapshot]] = {}

    field_length_half = _DEFAULT_FIELD_LENGTH_HALF
    field_width_half = _DEFAULT_FIELD_WIDTH_HALF
    geometry_loaded = False
    primary_uuid: str | None = None
    start_ts_ns: int | None = None
    end_ts_ns: int = 0
    last_command_counter: int = -1
    yellow_name = ""
    blue_name = ""

    for ts_ns, msg_id, payload in iter_messages(path):
        msg_counts[msg_id] = msg_counts.get(msg_id, 0) + 1
        if start_ts_ns is None:
            start_ts_ns = ts_ns
        end_ts_ns = ts_ns
        t = (ts_ns - (start_ts_ns or ts_ns)) / 1e9

        if msg_id == _MSG_VISION_2014 and not geometry_loaded:
            pkt = SSL_WrapperPacket()
            pkt.ParseFromString(payload)
            if pkt.HasField("geometry") and pkt.geometry.HasField("field"):
                fld = pkt.geometry.field
                field_length_half = fld.field_length / 2000.0
                field_width_half = fld.field_width / 2000.0
                geometry_loaded = True

        elif msg_id == _MSG_TRACKER_2020:
            pkt = TrackerWrapperPacket()
            try:
                pkt.ParseFromString(payload)
            except Exception:
                continue
            if primary_uuid is None:
                primary_uuid = pkt.uuid
            elif pkt.uuid != primary_uuid:
                continue
            if not pkt.HasField("tracked_frame"):
                continue
            frame = pkt.tracked_frame
            # ログヘッダのts_ns（UNIX時刻）を基準にする（frame.timestampはエポックが異なる場合がある）
            frame_ts_ns = ts_ns
            frame_t = t

            if frame.balls:
                ball = frame.balls[0]
                pos = ball.pos
                if ball.HasField("vel"):
                    vel = ball.vel
                    vx, vy = vel.x, vel.y
                else:
                    vx, vy = 0.0, 0.0
                speed = math.hypot(vx, vy)
                if speed < 0.05:
                    state = "STOPPED"
                elif pos.z > 0.02:
                    state = "FLYING"
                else:
                    state = "ROLLING"
                has_kicked = frame.HasField("kicked_ball")
                ball_timeline.append(
                    BallSnapshot(
                        ts_ns=frame_ts_ns,
                        t=frame_t,
                        x=pos.x,
                        y=pos.y,
                        z=pos.z,
                        vx=vx,
                        vy=vy,
                        speed=speed,
                        state=state,
                        has_kicked_ball=has_kicked,
                    )
                )

            for robot in frame.robots:
                team_val = robot.robot_id.team  # 1=YELLOW, 2=BLUE
                team = "yellow" if team_val == 1 else "blue"
                rid = robot.robot_id.id
                pos = robot.pos
                orient = robot.orientation
                vel = robot.vel if robot.HasField("vel") else None
                vx = vel.x if vel else 0.0
                vy = vel.y if vel else 0.0
                spd = math.hypot(vx, vy)
                vis = robot.visibility if robot.HasField("visibility") else 1.0  # noqa: SIM108
                key = (team, rid)
                if key not in robot_timeline:
                    robot_timeline[key] = []
                robot_timeline[key].append(
                    RobotSnapshot(
                        ts_ns=frame_ts_ns,
                        t=frame_t,
                        team=team,
                        robot_id=rid,
                        x=pos.x,
                        y=pos.y,
                        orientation=orient,
                        vx=vx,
                        vy=vy,
                        speed=spd,
                        visibility=vis,
                    )
                )

        elif msg_id == _MSG_REFEREE_2013:
            pkt = Referee()
            try:
                pkt.ParseFromString(payload)
            except Exception:
                continue
            if pkt.command_counter == last_command_counter:
                continue
            last_command_counter = pkt.command_counter
            stage = _STAGE_NAMES.get(pkt.stage, str(pkt.stage))
            command = _COMMAND_NAMES.get(pkt.command, str(pkt.command))
            if not yellow_name and pkt.yellow.name:
                yellow_name = pkt.yellow.name
            if not blue_name and pkt.blue.name:
                blue_name = pkt.blue.name
            game_event_types = [_ge_type_name(ge.type) for ge in pkt.game_events]
            referee_states.append(
                RefereeState(
                    ts_ns=ts_ns,
                    t=t,
                    stage=stage,
                    command=command,
                    command_counter=pkt.command_counter,
                    yellow_score=pkt.yellow.score,
                    blue_score=pkt.blue.score,
                    yellow_name=pkt.yellow.name,
                    blue_name=pkt.blue.name,
                    yellow_yellow_cards=len(pkt.yellow.yellow_card_times),
                    blue_yellow_cards=len(pkt.blue.yellow_card_times),
                    yellow_red_cards=pkt.yellow.red_cards,
                    blue_red_cards=pkt.blue.red_cards,
                    game_event_types=game_event_types,
                )
            )

    if start_ts_ns is None:
        start_ts_ns = 0
    duration = (end_ts_ns - start_ts_ns) / 1e9

    info = LogInfo(
        path=str(path),
        start_ts_ns=start_ts_ns,
        end_ts_ns=end_ts_ns,
        duration_sec=duration,
        msg_counts=msg_counts,
        yellow_team_name=yellow_name,
        blue_team_name=blue_name,
        field_length_m=field_length_half * 2,
        field_width_m=field_width_half * 2,
    )
    return LogData(
        info=info,
        referee_states=referee_states,
        ball_timeline=ball_timeline,
        robot_timeline=robot_timeline,
    )


def _ge_type_name(value: int) -> str:
    try:
        from ssl_gc_game_event_pb2 import GameEvent  # noqa: PLC0415

        return GameEvent.Type.Name(value)
    except (ValueError, KeyError):
        return f"GAME_EVENT_{value}"
