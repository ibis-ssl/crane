"""イベント検出."""

from dataclasses import dataclass, field
from typing import Any

from .metrics import speed_2d
from .models import BagData

EVENT_GOAL = "goal"
EVENT_PLAY_TRANSITION = "play"
EVENT_ROLE_CHANGE = "role"
EVENT_KICK = "kick"
EVENT_BALL_SPEED = "ball_speed"

ALL_EVENT_TYPES = [
    EVENT_GOAL,
    EVENT_PLAY_TRANSITION,
    EVENT_ROLE_CHANGE,
    EVENT_KICK,
    EVENT_BALL_SPEED,
]


@dataclass
class Event:
    """検出イベント."""

    timestamp_ns: int
    t: float
    event_type: str
    description: str
    details: dict[str, Any] = field(default_factory=dict)


def detect_events(
    bag_data: BagData,
    types: list[str] | None = None,
) -> list[Event]:
    """指定タイプのイベントを検出する.

    Args:
        bag_data: BagData
        types: 検出するイベントタイプのリスト。Noneの場合は全タイプ。

    Returns:
        Eventのリスト（時系列順）
    """
    target = set(types) if types is not None else set(ALL_EVENT_TYPES)
    all_events: list[Event] = []

    if EVENT_PLAY_TRANSITION in target:
        all_events.extend(detect_play_transitions(bag_data))
    if EVENT_ROLE_CHANGE in target:
        all_events.extend(detect_role_changes(bag_data))
    if EVENT_KICK in target:
        all_events.extend(detect_kick_events(bag_data))
    if EVENT_BALL_SPEED in target:
        all_events.extend(detect_ball_speed_spikes(bag_data))
    if EVENT_GOAL in target:
        all_events.extend(detect_goals(bag_data))

    all_events.sort(key=lambda e: e.timestamp_ns)
    return all_events


def detect_play_transitions(bag_data: BagData) -> list[Event]:
    """プレイ状態の遷移を検出する."""
    events = []
    prev_cmd = None
    for tm in bag_data.get("/play_situation"):
        cmd_name = tm.msg.command.name
        if cmd_name != prev_cmd:
            events.append(
                Event(
                    timestamp_ns=tm.timestamp_ns,
                    t=tm.t,
                    event_type=EVENT_PLAY_TRANSITION,
                    description=f"PLAY: {prev_cmd} -> {cmd_name}",
                    details={
                        "command": cmd_name,
                        "reason": tm.msg.reason_text,
                        "prev_command": prev_cmd,
                    },
                )
            )
            prev_cmd = cmd_name
    return events


def detect_role_changes(bag_data: BagData) -> list[Event]:
    """ロール割り当ての変化を検出する."""
    events = []
    prev_roles: dict[str, list[int]] = {}

    for tm in bag_data.get("/robot_select_results"):
        current_roles: dict[str, list[int]] = {}
        for r in tm.msg.results:
            current_roles[r.name] = list(r.selected_robots)

        if current_roles != prev_roles:
            changes = {}
            for role_name, robots in current_roles.items():
                if prev_roles.get(role_name) != robots:
                    changes[role_name] = {
                        "from": prev_roles.get(role_name, []),
                        "to": robots,
                    }
            if changes:
                desc_parts = [
                    f"{name}: {v['from']}->{v['to']}" for name, v in changes.items()
                ]
                events.append(
                    Event(
                        timestamp_ns=tm.timestamp_ns,
                        t=tm.t,
                        event_type=EVENT_ROLE_CHANGE,
                        description=f"ROLE: {', '.join(desc_parts)}",
                        details={"changes": changes, "all_roles": current_roles},
                    )
                )
            prev_roles = current_roles

    return events


def detect_kick_events(bag_data: BagData) -> list[Event]:
    """キックイベントを検出する（kick_power > 0 の立ち上がりエッジ）."""
    events = []
    prev_kicking: set[int] = set()

    for tm in bag_data.get("/robot_commands"):
        kicking_now: set[int] = set()
        for rc in tm.msg.robot_commands:
            if rc.kick_power > 0:
                kicking_now.add(rc.robot_id)

        # 新しいキック検出（立ち上がりエッジ）
        new_kicks = kicking_now - prev_kicking
        for robot_id in new_kicks:
            events.append(
                Event(
                    timestamp_ns=tm.timestamp_ns,
                    t=tm.t,
                    event_type=EVENT_KICK,
                    description=f"KICK: robot={robot_id}",
                    details={"robot_id": robot_id},
                )
            )

        prev_kicking = kicking_now

    return events


def detect_ball_speed_spikes(bag_data: BagData, threshold: float = 3.0) -> list[Event]:
    """ボール速度スパイクを検出する.

    Args:
        bag_data: BagData
        threshold: スパイク判定閾値（m/s）
    """
    events = []
    prev_above = False

    for tm in bag_data.get("/world_model"):
        ball = tm.msg.ball_info
        spd = speed_2d(ball.velocity.x, ball.velocity.y)
        above = spd >= threshold

        if above and not prev_above:
            events.append(
                Event(
                    timestamp_ns=tm.timestamp_ns,
                    t=tm.t,
                    event_type=EVENT_BALL_SPEED,
                    description=f"BALL_SPEED: {spd:.2f}m/s >= {threshold}m/s",
                    details={
                        "speed": spd,
                        "threshold": threshold,
                        "ball_x": ball.position.x,
                        "ball_y": ball.position.y,
                    },
                )
            )

        prev_above = above

    return events


def detect_goals(bag_data: BagData) -> list[Event]:
    """ゴールを検出する（ボールがゴールライン通過を検知）."""
    events = []

    # フィールド情報をWorldModelから取得
    world_msgs = bag_data.get("/world_model")
    if not world_msgs:
        return events

    # field_infoからフィールドサイズを取得
    first_msg = world_msgs[0].msg
    field_info = getattr(first_msg, "field_info", None)
    if field_info is None:
        return events

    half_length = field_info.x / 2.0  # field_length/2

    GOAL_HALF_WIDTH = 0.5  # ゴール幅の半分（1m）

    prev_in_goal = False

    for tm in world_msgs:
        ball = tm.msg.ball_info
        bx, by = ball.position.x, ball.position.y

        in_goal = abs(bx) >= half_length and abs(by) <= GOAL_HALF_WIDTH

        if in_goal and not prev_in_goal:
            side = "THEIR_GOAL" if bx > 0 else "OUR_GOAL"
            events.append(
                Event(
                    timestamp_ns=tm.timestamp_ns,
                    t=tm.t,
                    event_type=EVENT_GOAL,
                    description=f"GOAL: {side} ball=({bx:.2f},{by:.2f})",
                    details={"side": side, "ball_x": bx, "ball_y": by},
                )
            )

        prev_in_goal = in_goal

    return events
